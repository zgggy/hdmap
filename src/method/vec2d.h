#ifndef __VEC2D__
#define __VEC2D__

#include <cmath>
#include <string>
#include <tuple>

namespace hdmap {

#define EPSILON 1e-6

class Vec2D {
  public:
    double x, y;

  public:
    Vec2D() : x(0), y(0) {}
    Vec2D(double x, double y) : x(x), y(y) {}
    auto operator+(const Vec2D& other) -> Vec2D {
        return Vec2D(x + other.x, y + other.y);
    }
    auto operator-(const Vec2D& other) -> Vec2D {
        return Vec2D(x - other.x, y - other.y);
    }
    auto operator*(const Vec2D& other) -> double {
        return x * other.x + y * other.y;
    }
    auto operator*(const double other) -> Vec2D {
        return Vec2D(x * other, y * other);
    }

    template <class VecType>
    auto operator==(const VecType& other) -> bool {
        return (fabs(this->x - other.x) < EPSILON) and (fabs(this->y - other.y) < EPSILON);
    }
    template <class VecType>
    auto operator!=(const VecType& other) -> bool {
        return (fabs(this->x - other.x) > EPSILON) or (fabs(this->y - other.y) > EPSILON);
    }

    auto cross(const Vec2D& other) -> double {
        return x * other.x - y * other.y;
    }
    auto mod() -> double {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    auto angle_with(const Vec2D& other) -> double {
        double theta = atan2(other.y, other.x) - atan2(y, x); // 弧度
        if (theta > M_PI) theta -= 2 * M_PI;
        if (theta < -M_PI) theta += 2 * M_PI;
        return fabs(theta * 180 / M_PI); // 角度
    }

    virtual auto Str() -> std::string {
        return "V(" + std::to_string(x) + "," + std::to_string(y) + ")";
    }

    template <class VecType>
    auto near(const VecType& other, double distance) -> bool {
        return (fabs(this->x - other.x) < distance) and (fabs(this->y - other.y) < distance);
    }
};

class SimpPoint : public Vec2D {
  public:
    double theta_;
    auto   theta() -> double {
        return std::fmod(theta_ + 2 * M_PI, 2 * M_PI);
    }

    SimpPoint() : Vec2D(), theta_(0) {}
    SimpPoint(const Vec2D& some_vec) : Vec2D(some_vec.x, some_vec.y), theta_(0) {}
    SimpPoint(double x, double y) : Vec2D(x, y), theta_(0) {}
    SimpPoint(double x, double y, double h) : Vec2D(x, y), theta_(h) {}

    auto Vec() -> Vec2D {
        return Vec2D{x, y};
    }

    virtual auto Str() -> std::string {
        return "S(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta()) + ")";
    }
};

class Point : public SimpPoint {
  public:
    double c, v;

    Point() : SimpPoint(), c(0), v(0) {}
    Point(double x, double y, double theta, double c) : SimpPoint(x, y, theta), c(c), v(0) {}
    Point(double x, double y, double theta, double c, double v) : SimpPoint(x, y, theta), c(c), v(v) {}

    auto Simp() -> SimpPoint {
        return SimpPoint{x, y, theta_};
    }

    virtual auto Str() -> std::string {
        return "P(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta()) + "," +
               std::to_string(c) + ")";
    }
};

class WayPoint : public Point {
  public:
    double t;            // 点的时间戳，在速度预测时使用
    double z;            // 点的高度
    double ramp;         // 点的坡度
    double left, right;  // 点的左右边界
    double direction{0}; // 点的运动方向 0 前进 1 后退
    double distance;
    WayPoint() : Point(), z(0), ramp(0), t(0) {}
    WayPoint(double x, double y, double theta, double c) : Point(x, y, theta, c) {}
    WayPoint(double x, double y, double z, double theta, double c, double v, double t, double left, double right,
             double distance, int direction)
            : Point(x, y, theta, c, v)
            , t(t)
            , z(z)
            , left(left)
            , right(right)
            , distance(distance)
            , direction(direction) {}
    WayPoint(double x, double y, double theta, double c, double v, double t) : Point(x, y, theta, c, v), t(t) {}
};

} // namespace hdmap
#endif // __VEC2D__