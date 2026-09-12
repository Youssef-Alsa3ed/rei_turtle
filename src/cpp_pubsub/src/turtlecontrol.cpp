
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include <chrono>
#include <memory>
#include <turtlesim/srv/detail/set_pen__struct.hpp>
#include <turtlesim/srv/detail/teleport_absolute__struct.hpp>
#include <vector>
#include <atomic>
#include "graphics/shape.h"

//math
#include <glm/glm.hpp>
using namespace std::chrono_literals;

enum class turtlestate { LIFTPEN, START, PUSHPEN, DRAW, DONE };

class TurtleController : public rclcpp::Node {
public:
    TurtleController(std::shared_ptr<Shape> shape)
        : Node("turtlecontroller"),
          shape(shape),
          contours(shape->GetContours()),
          width(shape->GetSpaceDimensions().x),
          height(shape->GetSpaceDimensions().y) {
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        timer = create_wall_timer(3ms, std::bind(&TurtleController::controlLoop, this));

        for (auto& v : contours) {
            for (auto& p : v) {
                RCLCPP_INFO(this->get_logger(), "point: (%d, %d)", p.x, p.y);
            }
        }
    }

private:
    std::shared_ptr<Shape> shape;

    glm::vec2 transfromPointToTurtleSpace(glm::vec2 p) {
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

    int currentContour = 0, currentPoint = 0;

    void controlLoop() {
        using enum turtlestate;
        if (state == DONE || isWorking) {
            return;
        }

        if (currentContour >= contours.size()) {
            RCLCPP_INFO(get_logger(), "Done drawing all contours");
            state = DONE;
            return;
        }
        glm::vec2 target = transfromPointToTurtleSpace(contours[currentContour][currentPoint]);
        glm::vec2 prev = target;
        if (currentPoint != 0) {
            prev = transfromPointToTurtleSpace(contours[currentContour][currentPoint - 1]);
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
                if (currentPoint >= contours[currentContour].size()) {
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

    std::atomic_bool isWorking{false};

private:
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::TimerBase::SharedPtr timer;
    turtlestate state = turtlestate::LIFTPEN;
    std::vector<std::vector<glm::vec2>> contours;
    double width;
    double height;
};
int main(int argc, char* argv[]) {
    for (int i = 0; i < argc; ++i)
        std::cout << i << ": " << argv[i] << '\n';

    rclcpp::init(argc, argv);
    std::shared_ptr<Shape> shape = std::make_shared<OpenCVContourShape>(std::string(argv[1]));
    auto triangle = std::make_shared<TriangularFractalShape>(5);
    auto turtle = std::make_shared<TurtleController>(shape);
    rclcpp::spin(turtle);
    rclcpp::shutdown();

    return 0;
}