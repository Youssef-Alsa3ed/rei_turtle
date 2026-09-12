#include "shape.h"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace glm;
OpenCVContourShape::OpenCVContourShape(const std::string& path) {
    cv::Mat image = cv::imread(path);
    if (image.empty()) {
        std::cerr << "Failed to load image\n";
    }

    width = image.cols;
    height = image.rows;
    cv::Mat gray, blurred;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_KCOS);

    //clean noise
    std::erase_if(contours, [](const auto& contour) { return contour.size() < 2; });

    std::cout << "contours to draw: " << contours.size() << '\n';

    cv::Mat preview = cv::Mat::zeros(image.size(), CV_8UC3);

    vectorizedContours.resize(contours.size());

    std::ranges::transform(contours, vectorizedContours.begin(), [](const auto& src_contour) {
        std::vector<vec2> dest_contour;
        dest_contour.reserve(src_contour.size());

        std::ranges::transform(src_contour, std::back_inserter(dest_contour), [](const cv::Point& p) {
            return vec2(static_cast<float>(p.x), static_cast<float>(p.y));
        });

        return dest_contour;
    });
}

const std::vector<std::vector<vec2>>& OpenCVContourShape::GetContours() const {
    return vectorizedContours;
}

TriangularFractalShape::TriangularFractalShape(int depth) {
    using namespace glm;

    int num_triangles = std::pow(3, depth);

    contours.reserve(num_triangles);

    vec2 A = vec2(0.5, 0.0);
    vec2 B = vec2(0.0, 1.0);
    vec2 C = vec2(1.0, 1.0);

    GenerateSierpinskiTriangle(A, B, C, depth);
}

const std::vector<std::vector<vec2>>& TriangularFractalShape::GetContours() const {
    return contours;
}

const vec2 OpenCVContourShape::GetSpaceDimensions() {
    return vec2(static_cast<float>(width), static_cast<float>(height));
}

const vec2 TriangularFractalShape::GetSpaceDimensions() {
    return vec2(1.0, 1.0); // 0-1 space
}

glm::vec2 TriangularFractalShape::midpoint(vec2 p1, vec2 p2) {
    return vec2((p1 + p2) / 2.0f);
}

void TriangularFractalShape::GenerateSierpinskiTriangle(vec2 p1, vec2 p2, vec2 p3, int currentDepth) {
    if (currentDepth == 0) {
        contours.push_back({p1, p2, p3, p1});
        return;
    }

    vec2 m12 = midpoint(p1, p2);
    vec2 m23 = midpoint(p2, p3);
    vec2 m31 = midpoint(p3, p1);

    GenerateSierpinskiTriangle(p1, m12, m31, currentDepth - 1);
    GenerateSierpinskiTriangle(m12, p2, m23, currentDepth - 1);
    GenerateSierpinskiTriangle(m31, m23, p3, currentDepth - 1);
}