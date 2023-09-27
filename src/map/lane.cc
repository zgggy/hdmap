#include "map.h"

namespace hdmap {

auto BaseLane::L(double s) -> double {
    return planner::poly_5(parameters_, s, 0);
}
auto BaseLane::dL(double s) -> double {
    return planner::poly_5(parameters_, s, 1);
}
auto BaseLane::ddL(double s) -> double {
    return planner::poly_5(parameters_, s, 2);
}

auto BaseLane::RoadLength() -> double {
    return end_s_ - start_s_;
}

auto BaseLane::GetPoint(double s) -> planner::Point {
    auto p  = road_->ref_traj_.GetPoint(s);
    auto l  = L(s);
    auto x2 = p.x + cos(p.theta() + M_PI / 2) * l;
    auto y2 = p.y + sin(p.theta() + M_PI / 2) * l;
    auto c2 = p.c == 0 ? 0 : 1 / (1 / p.c - l);

    // std::cout<< l << " " << x2 << " " << y2 << std::endl;
    return planner::Point{x2, y2, p.theta() /* TODO 刘旅帆公式 */, c2};
}

auto BaseLane::Start() -> planner::Point {
    return GetPoint(start_s_);
}

auto BaseLane::End() -> planner::Point {
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

auto BaseLane::SampleTraj(double waypoint_interval) -> vector<planner::Point> {
    auto [x, y, t, c] = SampleAll(waypoint_interval);
    auto result       = vector<planner::Point>{};
    for (int i = 0; i != x.size(); i++) {
        result.emplace_back(planner::Point{x[i], y[i], t[i], c[i]});
    }
    return result;
}

auto BaseLane::NearestWith(planner::SimpPoint point) -> tuple<double, double> {}

auto Lane::LaneID::Str() -> std::string {
    return std::to_string(road) + std::to_string(section) + std::to_string(group) + std::to_string(lane) +
           std::to_string(unit);
}

bool Lane::LaneID::operator<(const LaneID& other) const {
    if (this->road != other.road)
        return this->road < other.road;
    else if (this->section != other.section)
        return this->section < other.section;
    else if (this->group != other.group)
        return this->group < other.group;
    else if (this->lane != other.lane)
        return this->lane < other.lane;
    else if (this->unit != other.unit)
        return this->unit < other.unit;
    else
        return false;
}

bool Lane::LaneID::operator>(const LaneID& other) const {
    return !(*this < other);
}

bool Lane::LaneID::operator==(const LaneID& other) const {
    return road == other.road and section == other.section and group == other.group and lane == other.lane and
           unit == other.unit;
}

bool Lane::LaneID::operator!=(const LaneID& other) const {
    return !(*this == other);
}

auto Solid::SolidID::Str() -> std::string {
    return std::to_string(road) + std::to_string(section) + std::to_string(mid_group);
}

bool Solid::SolidID::operator<(const SolidID& other) const {
    if (this->road != other.road)
        return this->road < other.road;
    else if (this->section != other.section)
        return this->section < other.section;
    else if (this->mid_group != other.mid_group)
        return this->mid_group < other.mid_group;
    else
        return false;
}

bool Solid::SolidID::operator>(const SolidID& other) const {
    return !(*this < other);
}

bool Solid::SolidID::operator==(const SolidID& other) const {
    return road == other.road and section == other.section and mid_group == other.mid_group;
}

bool Solid::SolidID::operator!=(const SolidID& other) const {
    return !(*this == other);
}

Lane::Lane(std::shared_ptr<Map> map, std::shared_ptr<Road> road, int section_num, int part_num, int lane_num,
           PolyPara parameters, int parameter_type) {
    map_         = map;
    road_        = road;
    id_          = Lane::LaneID{road_->id_, section_num, part_num, lane_num, 0};
    auto section = road_->sections_[id_.section];
    start_s_     = section.first;
    end_s_       = section.second;
    if (parameter_type == Lane::PARAMETER_TYPE::SE)
        parameters_ = planner::solve_poly_5(start_s_, end_s_, parameters);
    else
        parameters_ = parameters;
}

auto Lane::Cut(std::initializer_list<double> unit_list) -> vector<Lane> {
    auto units = vector<double>{unit_list};
    units.emplace(units.begin(), start_s_);
    units.emplace_back(end_s_);
    auto lanes = vector<Lane>{};
    for (int i = 0; i != units.size() - 1; ++i) {
        auto lane     = Lane(map_, road_, id_.section, id_.group, id_.lane, parameters_, Lane::PARAMETER_TYPE::AB);
        lane.id_.unit = i;
        lane.start_s_ = units[i];
        lane.end_s_   = units[i + 1];
        lanes.emplace_back(lane);
    }
    return lanes;
}

Solid::Solid(std::shared_ptr<Map> map, std::shared_ptr<Road> road, int section_num, double mid_part,
             PolyPara parameters, int parameter_type) {
    map_         = map;
    road_        = road;
    id_          = Solid::SolidID{road_->id_, section_num, mid_part};
    auto section = road_->sections_[id_.section];
    start_s_     = section.first;
    end_s_       = section.second;
    if (parameter_type == Lane::PARAMETER_TYPE::SE)
        parameters_ = planner::solve_poly_5(start_s_, end_s_, parameters);
    else if (parameter_type == Lane::PARAMETER_TYPE::AB)
        parameters_ = parameters;
}

} // namespace hdmap