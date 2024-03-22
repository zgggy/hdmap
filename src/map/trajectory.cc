#include "map.h"

namespace hdmap {

auto Trajectory::Update(vector<Segment> segments) -> void {
    segments_ = segments;
    length_   = 0.0;
    for (auto seg : segments_) length_ += seg.Length();
    sections_ = {std::make_pair(0, length_)};
}

auto Trajectory::Update(Segment segment) -> void {
    if (segment.type_ == Segment::SEGMENT_TYPE::LINE) {
        segments_.emplace_back(segment);
        length_ += segment.Length();
    } else if (segment.type_ == Segment::SEGMENT_TYPE::CLOTHOID)
        Update(segment.To3Segment());
    else
        std::cout << "segment type error: " << segment.type_ << std::endl;
    sections_ = {std::make_pair(0, length_)};
}

auto Trajectory::Cat(const Trajectory& trajectory) -> void {
    for (auto seg : trajectory.segments_) Update(seg);
}


auto Trajectory::Cut(std::initializer_list<double> sec_list) -> void {
    sections_.clear();
    auto segs = vector<double>{sec_list};
    sections_.emplace_back(std::make_pair(0, segs[0]));
    for (int i = 0; i != segs.size() - 1; ++i) {
        sections_.emplace_back(std::make_pair(segs[i], segs[i + 1]));
    }
    sections_.emplace_back(std::make_pair(segs.back(), length_));
}

auto Trajectory::Length() -> double {
    return length_;
}

auto Trajectory::GetValue(SAMPLE_TYPE sample_type, double s) -> double {
    auto ii = int{0};
    auto ss = s;
    for (int i = 0; i != segments_.size(); i++)
        if (ss > segments_[i].length_)
            ss -= segments_[i].length_;
        else {
            ii = i;
            break;
        }
    return segments_[ii].GetValue(sample_type, ss);
}

auto Trajectory::GetPoint(double s) -> Point {
    auto x = GetValue(SAMPLE_TYPE::X, s);
    auto y = GetValue(SAMPLE_TYPE::Y, s);
    auto t = GetValue(SAMPLE_TYPE::T, s);
    auto c = GetValue(SAMPLE_TYPE::C, s);
    return Point{x, y, t, c};
}

auto Trajectory::Sample(SAMPLE_TYPE sample_type, double waypoint_interval) -> vector<double> {
    double         s = 0;
    vector<double> result;
    while (s < length_) {
        result.emplace_back(GetValue(sample_type, s));
        s += waypoint_interval;
    }
    return result;
}

auto Trajectory::SampleAll(double waypoint_interval)
    -> tuple<vector<double>, vector<double>, vector<double>, vector<double>> {
    auto x = Sample(SAMPLE_TYPE::X, waypoint_interval);
    auto y = Sample(SAMPLE_TYPE::Y, waypoint_interval);
    auto t = Sample(SAMPLE_TYPE::T, waypoint_interval);
    auto c = Sample(SAMPLE_TYPE::C, waypoint_interval);
    return make_tuple(x, y, t, c);
}

auto Trajectory::SampleTraj(double waypoint_interval) -> vector<Point> {
    auto [x, y, t, c] = SampleAll(waypoint_interval);
    auto result       = vector<Point>{};
    for (int i = 0; i != x.size(); i++) result.emplace_back(Point{x[i], y[i], t[i], c[i]});
    return result;
}

auto Trajectory::NearestWith(SimpPoint point) -> tuple<double, double> {
    auto curv = [this](double s) { return make_tuple(GetValue(SAMPLE_TYPE::X, s), GetValue(SAMPLE_TYPE::Y, s)); };
    return find_closest_point_on_curve(curv, point.x, point.y, 0, length_);
}

} // namespace hdmap