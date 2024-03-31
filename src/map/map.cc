#include "map.h"

namespace hdmap {

void Map::AddRoad(int road_id) {
    auto road = Road(road_id, shared_from_this());
    roads_.insert_or_assign(road_id, road);
}

void Map::AddLane(int road_id, int traj_id, int section_id, int group_id, std::initializer_list<double> parameters,
                  int param_type) {
    vector<double> paras = parameters;
    PolyPara       polyparas;
    if (param_type == BaseLane::PARAMETER_TYPE::F and paras.size() == 1)
        polyparas = make_tuple(0, 0, 0, 0, 0, paras[0]);
    else
        polyparas = make_tuple(paras[0], paras[1], paras[2], paras[3], paras[4], paras[5]);
    auto lane_num = 0;
    for (const auto& [_, lane] : lanes_)
        if (lane.road_id_ == road_id and lane.traj_id_ == traj_id and lane.section_id_ == section_id and
            lane.group_id_ == group_id)
            lane_num++;

    auto lane = Lane(shared_from_this(), road_id, traj_id, section_id, group_id, lane_num, polyparas, param_type);
    lanes_.emplace(lane.lane_full_id(), lane);
}

auto Map::AtLane(SimpPoint point) -> tuple<Lane::LaneID, double> {
    auto nearest_id   = Lane::LaneID{-1, -1, -1, -1, -1, -1};
    auto nearest_s    = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& [lane_full_id, lane] : lanes_) {
        auto [s, distance] = lane.NearestWith(point);
        if (distance < min_distance) {
            nearest_id   = lane.lane_full_id();
            nearest_s    = s;
            min_distance = distance;
        }
    }
    return make_tuple(nearest_id, nearest_s);
}

} // namespace hdmap