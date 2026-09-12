#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <turtlesim/srv/detail/set_pen__struct.hpp>
#include <turtlesim/srv/detail/teleport_absolute__struct.hpp>
#include <vector>
#include <atomic> 

using namespace std::chrono_literals;


enum class turtlestate {
    LIFTPEN, START, PUSHPEN, DRAW, DONE
};

class TurtleController : public rclcpp::Node
{
public:
    TurtleController(const std::vector<std::vector<cv::Point>>& extracted_contours, double img_width, double img_height)
        : Node("turtlecontroller"), contours(extracted_contours), width(img_width), height(img_height)
    {
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        timer = create_wall_timer(3ms, std::bind(&TurtleController::controlLoop, this));

        // for(auto& v : contours) {
        //     for(auto& p : v){
        //         RCLCPP_INFO(this->get_logger(), "point: (%d, %d)", p.x, p.y);
        //     }
        // }
    }

private:

cv::Point2d transfromPointToTurtleSpace(cv::Point p){
    double drawingSpace = 10.0; //clamp the space under 11 to avoid collisions with space boundaries
    double offsetX = (11.0 - drawingSpace) / (2.0);
    double offsetY = (11.0 - drawingSpace) / (2.0);

    double x = offsetX + ((double)p.x / width * drawingSpace);
    double normalized_y = (double)p.y / height;
    double y = offsetY + (drawingSpace - (normalized_y * drawingSpace)); //opencvs image coordinates starts from top left, while the turtle's y is bottom-up


    return cv::Point2d(x, y);
}



int currentContour = 0, currentPoint = 0;

void controlLoop(){
    using enum turtlestate;
    if(state == DONE || isWorking){
        return;
    }

    if(currentContour >= contours.size()){
        RCLCPP_INFO(get_logger(), "Done drawing all contours");
        state = DONE;
        return;
    }
    cv::Point2d target = transfromPointToTurtleSpace(contours[currentContour][currentPoint]);
    switch(state){
        case LIFTPEN:
            setPen(true);
            state = START;
            break;
        case START:
            teleport(target);
            state = PUSHPEN;
            currentPoint++;
            break;
        case PUSHPEN:
            setPen(false);
            state = DRAW;
            break;
        case DRAW:
            teleport(target);
            currentPoint++;
            if(currentPoint >= contours[currentContour].size()){
                currentPoint = 0;
                currentContour++;
                state = LIFTPEN;
            }
            break;
        case DONE:
            break;
    }

}


void teleport(cv::Point2d p){
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

    request->x = p.x;
    request->y = p.y;
    request->theta = 0.0;

    isWorking = true;
    teleport_client->async_send_request(request, [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future) {
        this->isWorking = false;
    });
}

void setPen(bool lift){

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 255;
    request->g = 255;
    request->b = 255;
    request->width = 1;
    request->off = lift ? 1 : 0;

    isWorking = true;
    
    pen_client->async_send_request(request, [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future){
        this->isWorking = false; // reset isWorking once done
    });
}

std::atomic_bool isWorking{false};
private:
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::TimerBase::SharedPtr timer;
    turtlestate state = turtlestate::LIFTPEN;
    std::vector<std::vector<cv::Point>> contours;
    double width;
    double height;

};
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: ros2 run cpp_pubsub turtlecontrol <path_to_image>\n";
        return 1;
    }

    for (int i = 0; i < argc; ++i)
        std::cout << i << ": " << argv[i] << '\n';

    cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
        std::cerr << "Failed to load image\n";
        return 1;
    }

    cv::Mat gray, blurred;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        edges, 
        contours, 
        cv::RETR_LIST, 
        cv::CHAIN_APPROX_TC89_KCOS 
    );

    //clean noise
    std::erase_if(contours, [](const auto& contour) {
        return contour.size() < 2; 
    });

    std::cout << "contours to draw: " << contours.size() << '\n';

    cv::Mat preview = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::drawContours(preview, contours, -1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("Turtle Path Preview", preview);
    rclcpp::init(argc, argv);
    auto turtle = std::make_shared<TurtleController>(contours, image.cols, image.rows);
    rclcpp::spin(turtle);
    rclcpp::shutdown();

    return 0;
}