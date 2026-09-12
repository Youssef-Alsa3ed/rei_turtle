#ifndef SHAPE
#define SHAPE

#include <string>
#include <vector>
#include <glm/glm.hpp>

// Shape Coordinate frame illustration, the turtle has to invert the y-axis besides transforming the points to it's local space
//  *-----> X
//  |
//  |
//  |
//  Y

using namespace glm;
class Shape {
public:
    const virtual std::vector<std::vector<vec2>>& GetContours() const = 0;

    const virtual vec2 GetSpaceDimensions() = 0;
    ~Shape() = default;
};

class OpenCVContourShape : public Shape {
public:
    const std::vector<std::vector<vec2>>& GetContours() const override;

    OpenCVContourShape(const std::string& path);

    const vec2 GetSpaceDimensions() override;

private:
    std::vector<std::vector<vec2>> vectorizedContours;

    int width, height;
};

class TriangularFractalShape : public Shape {
public:
    const std::vector<std::vector<vec2>>& GetContours() const override;

    const vec2 GetSpaceDimensions() override;
    TriangularFractalShape(int depth);

private:
    std::vector<std::vector<vec2>> contours;
    vec2 midpoint(vec2 p1, vec2 p2);
    void GenerateSierpinskiTriangle(vec2 p1, vec2 p2, vec2 p3, int currentDepth);
};

#endif