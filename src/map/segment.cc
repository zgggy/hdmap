#include "map.h"

namespace hdmap {

Segment::Segment(SEGMENT_TYPE type, Point start, Point end) : start_(start), end_(end), type_(type) {
    if (type_ == Segment::SEGMENT_TYPE::LINE) {
        length_       = sqrt(pow(end_.y - start_.y, 2) + pow(end_.x - start_.x, 2));
        start_.theta_ = atan2(end_.y - start_.y, end_.x - start_.x);
        end_.theta_   = start_.theta();
        start_.c      = 0;
        end_.c        = 0;
    } else if (type_ == Segment::SEGMENT_TYPE::CLOTHOID) {
        g2_.build(start_.x, start_.y, start_.theta(), start_.c, end_.x, end_.y, end_.theta(), end_.c);
        length_ = g2_.totalLength();
    }
}

Segment::Segment(Point start, double dk, double s) : start_(start) {
    spiral_.build(start.x, start.y, start.theta(), start.c, dk, s);
    length_ = spiral_.length();
    end_    = Point(spiral_.xEnd(), spiral_.yEnd(), spiral_.thetaEnd(), spiral_.kappaEnd());
    if (start.c == 0.0 && dk == 0.0) {
        type_ = LINE;
    } else {
        type_ = CLOTHOID;
    }
}

auto Segment::To3Segment() -> std::vector<Segment> {
    auto s1 =
        Segment(Point{g2_.getS0().xBegin(), g2_.getS0().yBegin(), g2_.getS0().thetaBegin(), g2_.getS0().kappaBegin()},
                g2_.getS0().dkappa(), g2_.getS0().length());
    auto s2 =
        Segment(Point{g2_.getSM().xBegin(), g2_.getSM().yBegin(), g2_.getSM().thetaBegin(), g2_.getSM().kappaBegin()},
                g2_.getSM().dkappa(), g2_.getSM().length());
    auto s3 =
        Segment(Point{g2_.getS1().xBegin(), g2_.getS1().yBegin(), g2_.getS1().thetaBegin(), g2_.getS1().kappaBegin()},
                g2_.getS1().dkappa(), g2_.getS1().length());
    return {s1, s2, s3};
}

auto Segment::Length() -> double {
    std::cout << "Segment length_: " << length_ << std::endl;
    return length_;
}

auto Segment::GetValue(SAMPLE_TYPE sample_type, double s) -> double {
    if (type_ == Segment::SEGMENT_TYPE::LINE)
        if (sample_type == SAMPLE_TYPE::X)
            return start_.x + cos(start_.theta()) * s;
        else if (sample_type == SAMPLE_TYPE::Y)
            return start_.y + sin(start_.theta()) * s;
        else if (sample_type == SAMPLE_TYPE::T)
            return start_.theta();
        else if (sample_type == SAMPLE_TYPE::C)
            return 0;
        else {
            std::cout << "No such SAMPLE_TYPE as [" << sample_type << "]" << std::endl;
            return MAXFLOAT;
        }
    else if (type_ == Segment::SEGMENT_TYPE::CLOTHOID)
        if (sample_type == SAMPLE_TYPE::X)
            return g2_.X(s);
        else if (sample_type == SAMPLE_TYPE::Y)
            return g2_.Y(s);
        else if (sample_type == SAMPLE_TYPE::T)
            return g2_.theta(s);
        else if (sample_type == SAMPLE_TYPE::C)
            return g2_.theta_D(s);
        else {
            std::cout << "No such SAMPLE_TYPE as [" << sample_type << "]" << std::endl;
            return MAXFLOAT;
        }
    else
        return MAXFLOAT;
}

auto Segment::GetPoint(double s) -> Point {
    auto x = GetValue(SAMPLE_TYPE::X, s);
    auto y = GetValue(SAMPLE_TYPE::Y, s);
    auto t = GetValue(SAMPLE_TYPE::T, s);
    auto c = GetValue(SAMPLE_TYPE::C, s);
    return Point{x, y, t, c};
}

auto Segment::Sample(SAMPLE_TYPE sample_type, double waypoint_interval) -> vector<double> {
    double         s = 0;
    vector<double> result;
    while (s < length_) {
        result.emplace_back(GetValue(sample_type, s));
        s += waypoint_interval;
    }
    return result;
}

auto Segment::SampleAll(double waypoint_interval)
    -> tuple<vector<double>, vector<double>, vector<double>, vector<double>> {
    auto x = Sample(SAMPLE_TYPE::X, waypoint_interval);
    auto y = Sample(SAMPLE_TYPE::Y, waypoint_interval);
    auto t = Sample(SAMPLE_TYPE::T, waypoint_interval);
    auto c = Sample(SAMPLE_TYPE::C, waypoint_interval);
    return make_tuple(x, y, t, c);
}

auto Segment::SampleTraj(double waypoint_interval) -> vector<Point> {
    auto [x, y, t, c] = SampleAll(waypoint_interval);
    auto result       = vector<Point>{};
    for (int i = 0; i != x.size(); i++) {
        result.emplace_back(Point{x[i], y[i], t[i], c[i]});
    }
    return result;
}

} // namespace hdmap