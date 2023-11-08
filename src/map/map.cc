#include "map.h"

namespace hdmap {

void Map::AddRoad(int road_id) {
    auto road = std::make_shared<Road>(road_id, shared_from_this());
    roads_.emplace(road_id, road);
}

void Map::AddLane(int road_id, int section_num, int part_num, std::initializer_list<double> parameters,
                  int param_type) {
    vector<double> paras = parameters;
    PolyPara       polyparas;
    if (param_type == BaseLane::PARAMETER_TYPE::F and paras.size() == 1)
        polyparas = make_tuple(0, 0, 0, 0, 0, paras[0]);
    else
        polyparas = make_tuple(paras[0], paras[1], paras[2], paras[3], paras[4], paras[5]);
    auto lane_num = 0;
    for (auto it = lanes_.begin(); it != lanes_.end(); it++)
        if (it->first.road == road_id and it->first.section == section_num and it->first.group == part_num) lane_num++;

    auto lane = std::make_shared<Lane>(shared_from_this(), roads_[road_id], section_num, part_num, lane_num, polyparas,
                                       param_type);
    lanes_.emplace(lane->id_, lane);
}

void Map::AddSolid(int road_id, int section_num, double mid_part, std::initializer_list<double> parameters,
                   int param_type) {
    vector<double> paras = parameters;
    PolyPara       polyparas;
    if (param_type == BaseLane::PARAMETER_TYPE::F and paras.size() == 1)
        polyparas = make_tuple(0, 0, 0, 0, 0, paras.back());
    else
        polyparas = make_tuple(paras[0], paras[1], paras[2], paras[3], paras[4], paras[5]);

    auto solid =
        std::make_shared<Solid>(shared_from_this(), roads_[road_id], section_num, mid_part, polyparas, param_type);
    solids_.emplace(solid->id_, solid);
}

auto Map::AtRoad(SimpPoint point) -> tuple<int, double> {
    auto nearest_id   = int{-1};
    auto nearest_s    = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& roaditem : roads_) {
        auto road          = roaditem.second;
        auto [s, distance] = road->ref_traj_.NearestWith(point);
        if (distance < min_distance) {
            nearest_id   = road->id_;
            nearest_s    = s;
            min_distance = distance;
        }
    }
    return make_tuple(nearest_id, nearest_s);
}

auto Map::AtRoutingRoad(SimpPoint point, vector<int> road_id_set) -> tuple<int, double> {
    auto nearest_id   = int{-1};
    auto nearest_s    = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& roaditem : road_id_set) {
        auto road          = roads_[roaditem];
        auto [s, distance] = road->ref_traj_.NearestWith(point);
        if (distance < min_distance) {
            nearest_id   = road->id_;
            nearest_s    = s;
            min_distance = distance;
        }
    }
    return make_tuple(nearest_id, nearest_s);
}

auto Map::AtRoadPtr(SimpPoint point) -> std::shared_ptr<Road> {
    auto [nearest_id, nearest_s] = AtRoad(point);
    return roads_[nearest_id];
}

auto Map::AtLane(SimpPoint point) -> tuple<Lane::LaneID, double> {
    auto nearest_id                        = Lane::LaneID{-1, -1, -1, -1, -1};
    auto nearest_s                         = double{-1};
    auto min_distance                      = MAXFLOAT;
    auto [nearest_road_id, nearest_road_s] = AtRoad(point);
    for (auto& laneitem : lanes_)
        if (laneitem.first.road == nearest_road_id) {
            auto lane          = laneitem.second;
            auto [s, distance] = lane->NearestWith(point);
            if (distance < min_distance) {
                nearest_id   = lane->id_;
                nearest_s    = s;
                min_distance = distance;
            }
        }
    return make_tuple(nearest_id, nearest_s);
}

auto Map::AtLanePtr(SimpPoint point) -> std::shared_ptr<Lane> {
    auto [nearest_id, nearest_s] = AtLane(point);
    return lanes_[nearest_id];
}

auto Map::AtLanesByS(int road_id, double s) -> vector<std::shared_ptr<Lane>> {
    auto road    = roads_[road_id];
    auto sec_num = 0;
    for (auto sec : road->sections_) {
        if (sec.first < s and s < sec.second) break;
        sec_num++;
    }
    auto laneptrs = vector<std::shared_ptr<Lane>>{};
    for (const auto& laneitem : lanes_)
        if (laneitem.first.road == road_id and laneitem.first.section == sec_num)
            laneptrs.emplace_back(laneitem.second);
    return laneptrs;
}

} // namespace hdmap