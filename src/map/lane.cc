#include "map.h"

namespace hdmap {

BaseLane::BaseLane(std::shared_ptr<Map> map, int road_id, int traj_id, int section_id, PolyPara parameters,
                   int parameter_type)
        : map_(map), road_id_(road_id), traj_id_(traj_id), section_id_(section_id) {
    auto section = map_->roads_.at(road_id_).ref_trajs_.at(traj_id_).sections_[section_id_];
    start_s_     = section.first;
    end_s_       = section.second;
    if (parameter_type == Lane::PARAMETER_TYPE::SE)
        parameters_ = solve_poly_5(start_s_, end_s_, parameters);
    else
        parameters_ = parameters;
}

auto BaseLane::L(double s) -> double {
    return poly_5(parameters_, s, 0);
}
auto BaseLane::dL(double s) -> double {
    return poly_5(parameters_, s, 1);
}
auto BaseLane::ddL(double s) -> double {
    return poly_5(parameters_, s, 2);
}

auto BaseLane::RoadLength() -> double {
    return end_s_ - start_s_;
}

auto BaseLane::GetPoint(double s) -> Point {
    auto p  = map_->roads_.at(road_id_).ref_trajs_.at(traj_id_).GetPoint(s);
    auto l  = L(s);
    auto x2 = p.x + cos(p.theta() + M_PI / 2) * l;
    auto y2 = p.y + sin(p.theta() + M_PI / 2) * l;
    auto c2 = p.c == 0 ? 0 : 1 / (1 / p.c - l);
    return Point{x2, y2, p.theta() /* TODO theta 是有变化的 */, c2};
}

auto BaseLane::Start() -> Point {
    return GetPoint(start_s_);
}

auto BaseLane::End() -> Point {
    return GetPoint(end_s_);
}

auto BaseLane::SampleAll(double waypoint_interval)
    -> tuple<vector<double>, vector<double>, vector<double>, vector<double>> {
    auto           s = start_s_;
    vector<double> xx, yy, tt, cc;
    while (s < end_s_) {
        auto p = GetPoint(s);
        xx.emplace_back(p.x);
        yy.emplace_back(p.y);
        tt.emplace_back(p.theta());
        cc.emplace_back(p.c);
        s += waypoint_interval;
    }
    return make_tuple(xx, yy, tt, cc);
}

auto BaseLane::SampleTraj(double waypoint_interval) -> vector<Point> {
    auto [x, y, t, c] = SampleAll(waypoint_interval);
    auto result       = vector<Point>{};
    for (int i = 0; i != x.size(); i++) {
        result.emplace_back(Point{x[i], y[i], t[i], c[i]});
    }
    return result;
}

auto BaseLane::NearestWith(SimpPoint point) -> tuple<double, double> {
    auto curv = [this](double s) {
        auto p = GetPoint(s);
        return make_tuple(p.x, p.y);
    };
    return find_closest_point_on_curve(curv, point.x, point.y, start_s_, end_s_);
}


Lane::Lane(std::shared_ptr<Map> map, int road_id, int traj_id, int section_id, int group_id, int lane_id,
           PolyPara parameters, int parameter_type)
        : BaseLane(map, road_id, traj_id, section_id, parameters, parameter_type)
        , group_id_(group_id)
        , lane_id_(lane_id)
        , unit_id_(0) {}

auto Lane::Cut(std::initializer_list<double> unit_list) -> std::vector<Lane> {
    auto units = vector<double>{unit_list};
    units.emplace(units.begin(), start_s_);
    units.emplace_back(end_s_);
    auto lanes = vector<Lane>{};
    for (int i = 0; i != units.size() - 1; ++i) {
        auto lane =
            Lane(map_, road_id_, traj_id_, section_id_, group_id_, lane_id_, parameters_, Lane::PARAMETER_TYPE::AB);
        unit_id_      = i;
        lane.start_s_ = units[i];
        lane.end_s_   = units[i + 1];
        lanes.emplace_back(lane);
    }
    return lanes;
}


} // namespace hdmap