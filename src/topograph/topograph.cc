#include "topograph.h"

namespace hdmap {

TopoGraph::TopoGraph(std::shared_ptr<hdmap::Map> map) {
    std::cout << "construct topo graph" << std::endl;
    origin_map_ = map;
    start_node_ = nullptr;
    end_node_   = nullptr;
    for (auto& [lane_full_id, lane] : origin_map_->lanes_) {
        std::cout << lane_full_id.road_id_ << lane_full_id.traj_id_ << lane_full_id.section_id_
                  << lane_full_id.group_id_ << lane_full_id.lane_id_ << std::endl;
        std::cout << "START S: " << lane.start_s_ << " " << lane.end_s_ << std::endl;
        auto node = std::make_shared<PathNode>(lane);
        std::cout << "make_shared" << std::endl;
        nodes_.emplace(node->lane_full_id(), node);
        std::cout << "emplace over~" << std::endl;
    }
    std::cout << "construct over~" << std::endl;
}

void TopoGraph::UpdateTopoGraph() {
    for (auto& node : nodes_) {
        node.second->successors_.clear();
        node.second->predecessors_.clear();
        node.second->sidecessors_.clear();
    }
    for (auto& [_, node1] : nodes_)
        for (auto& [_, node2] : nodes_)
            if (node1->lane_full_id() != node2->lane_full_id())
                if (node1->End() == node2->Start()) {
                    node1->successors_.emplace_back(node2->lane_full_id());
                    node2->predecessors_.emplace_back(node1->lane_full_id());
                } else if (node1->road_id_ == node2->road_id_ and node1->traj_id_ == node2->traj_id_ and
                           node1->section_id_ == node2->section_id_ and node1->group_id_ == node2->group_id_ and
                           node1->unit_id_ == node2->unit_id_ and abs(node1->lane_id_ - node2->lane_id_) == 1)
                    node1->sidecessors_.emplace_back(node2->lane_full_id());
}

void TopoGraph::SetStartAndEndNode(Lane::LaneID start_id, double start_s, Lane::LaneID end_id, double end_s) {
    if (start_id == end_id) {
        CutGroupByUnit(start_id.road_id_, start_id.traj_id_, start_id.section_id_, start_id.group_id_,
                       {fmin(start_s, end_s), fmax(start_s, end_s)});
        start_id.unit_id_ += start_s >= end_s ? 2 : 1;
        end_id.unit_id_ += end_s > start_s ? 1 : 0;
        start_node_ = nodes_[start_id];
    } else {
        CutGroupByUnit(start_id.road_id_, start_id.traj_id_, start_id.section_id_, start_id.group_id_, {start_s});
        start_id.unit_id_ += 1;
        start_node_ = nodes_[start_id];
        CutGroupByUnit(end_id.road_id_, end_id.traj_id_, end_id.section_id_, end_id.group_id_, {end_s});
    }
    end_node_ = nodes_[end_id];
}

void TopoGraph::CutGroupByUnit(int road_id, int traj_id, int section_id, int group_id,
                               std::initializer_list<double> unit_list) {
    std::map<Lane::LaneID, std::shared_ptr<PathNode>> nodes;
    for (auto& [node_full_id, node] : nodes_) {
        if (node_full_id.road_id_ == road_id and node_full_id.traj_id_ == traj_id and
            node_full_id.section_id_ == section_id and node_full_id.group_id_ == group_id) {
            auto new_lanes = node->Cut(unit_list);
            for (auto& l : new_lanes) {
                auto l_pointer = std::make_shared<PathNode>(l);
                nodes.insert_or_assign(l.lane_full_id(), l_pointer);
            }
        } else
            nodes.insert_or_assign(node_full_id, node);
    };
    nodes_ = nodes;
}

void TopoGraph::Print() {
    for (const auto& [_, l] : nodes_) {
        std::cout << "[" << l->lane_full_id().Str() << ":" << l->start_s_ << "," << l->end_s_ << "]"
                  << l->Start().Str() << "," << l->End().Str() << std::endl;
        for (auto& p : l->predecessors_) {
            std::cout << "< " << p.Str() << nodes_[p]->Start().Str() << "," << nodes_[p]->End().Str() << std::endl;
        }
        for (auto& s : l->successors_) {
            std::cout << "> " << s.Str() << nodes_[s]->Start().Str() << "," << nodes_[s]->End().Str() << std::endl;
        }
        for (auto& d : l->sidecessors_) {
            std::cout << "= " << d.Str() << nodes_[d]->Start().Str() << "," << nodes_[d]->End().Str() << std::endl;
        }
    }
    std::cout << "start: " << start_node_->lane_full_id().Str() << std::endl;
    std::cout << "end: " << end_node_->lane_full_id().Str() << std::endl;
}

/* TODO */
/* 根据输入的Point，为决策和规划模块提供一根长度为length的参考轨迹vector<Point> */
// auto TopoGraph::GetRefTraj(Lane::LaneID lane_id, double s, double length, double waypoint_interval) -> vector<Point>
// {
//     auto traj = nodes_[lane_id]->SampleTraj(waypoint_interval);
//     while (traj.size() < length / waypoint_interval) {
//         auto next_traj = vector<Point>{};
//         for (const auto& suc : nodes_[lane_id]->successors_) {
//             if (nodes_[suc]->is_best_) {
//                 next_traj = nodes_[suc]->SampleTraj(waypoint_interval);
//             }
//         }
//     }
// }

auto TopoGraph::AtNode(Point point) -> tuple<Lane::LaneID, double> {
    auto neareast_id  = Lane::LaneID{-1, -1, -1, -1, -1};
    auto neareast_s   = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& [node_full_id, node] : nodes_) {
        auto [s, distance] = node->NearestWith(point);
        if (distance < min_distance) {
            neareast_id  = node_full_id;
            neareast_s   = s;
            min_distance = distance;
        }
    }
    return make_tuple(neareast_id, neareast_s);
}

auto TopoGraph::AtNode(Lane::LaneID last_at_node, Point point) -> tuple<Lane::LaneID, double> {
    auto link = [this](Lane::LaneID node1id, Lane::LaneID node2id) {
        for (auto& sucnodeid : nodes_[node1id]->successors_) { // 后继
            if (sucnodeid == node2id) return true;
            for (auto& sucnodeid2 : nodes_[sucnodeid]->sidecessors_) // 后继的并行
                if (sucnodeid2 == node2id) return true;
        }
        for (auto& sucnodeid : nodes_[node1id]->sidecessors_) // 并行
            if (sucnodeid == node2id) return true;
        return false;
    };

    auto neareast_id  = Lane::LaneID{-1, -1, -1, -1, -1};
    auto neareast_s   = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& [node_full_id, node] : nodes_) {
        if (link(last_at_node, node_full_id)) {
            auto [s, distance] = node->NearestWith(point);
            if (distance < min_distance) {
                neareast_id  = node_full_id;
                neareast_s   = s;
                min_distance = distance;
            }
        }
    }
    return make_tuple(neareast_id, neareast_s);
}

} // namespace hdmap