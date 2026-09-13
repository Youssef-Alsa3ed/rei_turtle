//std
#include <chrono>
#include <memory>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <vector>
#include <atomic>

//ros
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/detail/set_pen__struct.hpp>
#include <turtlesim/srv/detail/teleport_absolute__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cpp_pubsub/srv/draw_shape.hpp>
//math
#include <glm/glm.hpp>

//shapes
#include "graphics/shape.h"

#define TURTLE_CENTER glm::vec2(5.544445f, 5.544445f)

using namespace std::chrono_literals;

enum class turtlestate { LIFTPEN, START, PUSHPEN, DRAW, DONE };

class TurtleController : public rclcpp::Node {
private:
    std::atomic_bool isWorking{false};
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client;

    rclcpp::TimerBase::SharedPtr timer;
    turtlestate state = turtlestate::DONE;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;
    rclcpp::Service<cpp_pubsub::srv::DrawShape>::SharedPtr draw_service;

    bool paused = false;

    void InitializeShape(std::shared_ptr<Shape> _shape) {
        shape = _shape;
        for (auto& v : shape->GetContours()) {
            for (auto& p : v) {
                RCLCPP_INFO(this->get_logger(), "point: (%f, %f)", p.x, p.y); // see what the actual point values are
            }
        }

        RCLCPP_INFO(get_logger(), "Contours to draw: %d", static_cast<int>(shape->GetContours().size()));
    }

public:
    TurtleController() : Node("turtlecontroller") {
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        clear_client = create_client<std_srvs::srv::Empty>("/clear");
        timer = create_wall_timer(3ms, std::bind(&TurtleController::controlLoop, this));

        pause_service = this->create_service<std_srvs::srv::Trigger>(
            "/turtle/pause",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                response->message = paused ? "Resuming the turtle" : "Pausing the turtle";
                response->success = true;
                paused = !paused;
                RCLCPP_INFO(get_logger(), "Pause service successfully ran.");
            });

        reset_service = this->create_service<std_srvs::srv::Trigger>(
            "turtle/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                response->message = "reseting turtle.";
                response->success = true;
                shape.reset();
                state = turtlestate::DONE;
                setPen(true);
                teleport(TURTLE_CENTER, 0.0f);
                currentContour = 0, currentPoint = 0;
                auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
                clear_client->async_send_request(clear_request);
                RCLCPP_INFO(get_logger(), "Canvas restored.");
            });

        draw_service = this->create_service<cpp_pubsub::srv::DrawShape>(
            "turtle/draw",
            [this](const std::shared_ptr<cpp_pubsub::srv::DrawShape::Request> request,
                   std::shared_ptr<cpp_pubsub::srv::DrawShape::Response> response) {
                shape.reset();
                std::string shape_name = request->shape_name;
                RCLCPP_INFO(get_logger(), "passed argument: %s", shape_name.c_str());

                response->success = true;
                if (shape_name == "sierpinski") {
                    state = turtlestate::LIFTPEN;
                    auto triangle = std::make_shared<TriangularFractalShape>(6);
                    InitializeShape(triangle);

                } else {
                    auto cvimg = std::make_shared<OpenCVContourShape>(shape_name);
                    if (cvimg->GetContours().empty()) {
                        RCLCPP_ERROR(get_logger(), "Passed name doesn't exist, either as a file path or shapename");
                        response->success = false;
                    } else {
                        state = turtlestate::LIFTPEN;
                        InitializeShape(cvimg);
                    }
                }

                response->message = response->success ? "Draw service started." : "Aborted.";
            });
        teleport_client->wait_for_service(1s);
        pen_client->wait_for_service(1s);
        clear_client->wait_for_service(1s);
    }

private:
    std::shared_ptr<Shape> shape;

    glm::vec2 transfromPointToTurtleSpace(glm::vec2 p) {
        double width = shape->GetSpaceDimensions().x, height = shape->GetSpaceDimensions().y;
        double drawingSpace = 10.0; //clamp the space under 11 to avoid collisions with space boundaries
        double offsetX = (11.0 - drawingSpace) / (2.0);
        double offsetY = (11.0 - drawingSpace) / (2.0);

        double x = offsetX + (p.x / width * drawingSpace);
        double normalized_y = p.y / height;
        double y = offsetY +
            (drawingSpace -
             (normalized_y *
              drawingSpace)); //opencvs image coordinates starts from top left, while the turtle's y is bottom-up

        return glm::vec2(x, y);
    }

    size_t currentContour = 0, currentPoint = 0;

    void controlLoop() {
        using enum turtlestate;

        if (paused) {
            RCLCPP_INFO(get_logger(), "turtle controller is paused.");
            return;
        }
        if (state == DONE || isWorking) {
            return;
        }

        if (currentContour >= shape->GetContours().size()) {
            RCLCPP_INFO(get_logger(), "Done drawing all contours");
            state = DONE;
            return;
        }
        glm::vec2 target = transfromPointToTurtleSpace(shape->GetContours()[currentContour][currentPoint]);
        glm::vec2 prev = target;
        if (currentPoint != 0) {
            prev = transfromPointToTurtleSpace(shape->GetContours()[currentContour][currentPoint - 1]);
        }

        double theta = std::atan2(target.y - prev.y, target.x - prev.x);
        switch (state) {
            case LIFTPEN:
                setPen(true);
                state = START;
                break;
            case START:
                teleport(target, theta);
                state = PUSHPEN;
                currentPoint++;
                break;
            case PUSHPEN:
                setPen(false);
                state = DRAW;
                break;
            case DRAW:
                teleport(target, theta);
                currentPoint++;
                if (currentPoint >= shape->GetContours()[currentContour].size()) {
                    currentPoint = 0;
                    currentContour++;
                    state = LIFTPEN;
                }
                break;
            case DONE:
                break;
        }
    }

    void teleport(glm::vec2 p, double theta) {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

        request->x = p.x;
        request->y = p.y;
        request->theta = theta;

        isWorking = true;
        teleport_client->async_send_request(
            request,
            [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future) { this->isWorking = false; });
    }

    void setPen(bool lift) {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = 255;
        request->g = 255;
        request->b = 255;
        request->width = 1;
        request->off = lift ? 1 : 0;

        isWorking = true;

        pen_client->async_send_request(request, [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
            this->isWorking = false; // reset isWorking once done
        });
    }
};

int main(int argc, char* argv[]) {
    for (int i = 0; i < argc; ++i)
        std::cout << i << ": " << argv[i] << '\n';

    rclcpp::init(argc, argv);
    auto turtle = std::make_shared<TurtleController>();
    rclcpp::spin(turtle);
    rclcpp::shutdown();

    return 0;
}