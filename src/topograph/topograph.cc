#include "topograph.h"

namespace hdmap {

TopoGraph::TopoGraph(std::shared_ptr<hdmap::Map> map) {
    std::cout << "construct topo graph" << std::endl;
    origin_map_ = map;
    start_node_ = nullptr;
    end_node_   = nullptr;
    for (auto& item : origin_map_->lanes_) {
        std::cout << item.first.road << item.first.section << item.first.group << item.first.lane << std::endl;
        std::cout << "START S: " << item.second->start_s_ << " " << item.second->end_s_ << std::endl;
        auto node = std::make_shared<PathNode>(item.second);
        std::cout << "make_shared" << std::endl;
        nodes_.emplace(node->id_, node);
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
    for (auto& node1 : nodes_) {
        auto l = node1.second;
        for (auto& node2 : nodes_) {
            auto l2 = node2.second;
            if (l->id_ != l2->id_)
                if (l->End() == l2->Start()) {
                    l->successors_.emplace_back(l2->id_);
                    l2->predecessors_.emplace_back(l->id_);
                } else if (l->id_.road == l2->id_.road and l->id_.section == l2->id_.section and
                           l->id_.group == l2->id_.group and l->id_.unit == l2->id_.unit and
                           abs(l->id_.lane - l2->id_.lane) == 1)
                    l->sidecessors_.emplace_back(l2->id_);
        }
    }
}

void TopoGraph::SetStartAndEndNode(Lane::LaneID start_id, double start_s, Lane::LaneID end_id, double end_s) {
    if (start_id == end_id) {
        CutGroup(start_id.road, start_id.section, start_id.group, {fmin(start_s, end_s), fmax(start_s, end_s)});
        start_id.unit += start_s >= end_s ? 2 : 1;
        end_id.unit += end_s > start_s ? 1 : 0;
        start_node_ = nodes_[start_id];
    } else {
        CutGroup(start_id.road, start_id.section, start_id.group, {start_s});
        start_id.unit += 1;
        start_node_ = nodes_[start_id];
        CutGroup(end_id.road, end_id.section, end_id.group, {end_s});
    }
    end_node_ = nodes_[end_id];
}

void TopoGraph::CutGroup(int road_id, int section_num, int group, std::initializer_list<double> unit_list) {
    std::map<Lane::LaneID, std::shared_ptr<PathNode>> nodes;
    for (auto& node : nodes_) {
        if (node.first.road == road_id and node.first.section == section_num and node.first.group == group) {
            auto new_lanes = node.second->Cut(unit_list);
            for (auto& l : new_lanes) {
                auto l_pointer = std::make_shared<PathNode>(std::make_shared<Lane>(l));
                nodes.emplace(l.id_, l_pointer);
            }
        } else
            nodes.emplace(node);
    };
    nodes_ = nodes;
}

void TopoGraph::Print() {
    for (const auto& lpair : nodes_) {
        auto l = lpair.second;
        std::cout << "[" << l->id_.Str() << ":" << l->start_s_ << "," << l->end_s_ << "]" << l->Start().Str() << ","
                  << l->End().Str() << std::endl;
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
    std::cout << "start: " << start_node_->id_.Str() << std::endl;
    std::cout << "end: " << end_node_->id_.Str() << std::endl;
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

auto TopoGraph::AtNode(int at_road, Point point) -> tuple<Lane::LaneID, double> {
    auto neareast_id  = Lane::LaneID{-1, -1, -1, -1, -1};
    auto neareast_s   = double{-1};
    auto min_distance = MAXFLOAT;
    for (auto& nodeitem : nodes_) {
        auto node = nodeitem.second;
        if (node->id_.road == at_road) {
            auto [s, distance] = node->NearestWith(point);
            if (distance < min_distance) {
                neareast_id  = node->id_;
                neareast_s   = s;
                min_distance = distance;
            }
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
    for (auto& nodeitem : nodes_) {
        if (link(last_at_node, nodeitem.first)) {
            auto node          = nodeitem.second;
            auto [s, distance] = node->NearestWith(point);
            if (distance < min_distance) {
                neareast_id  = node->id_;
                neareast_s   = s;
                min_distance = distance;
            }
        }
    }
    return make_tuple(neareast_id, neareast_s);
}

} // namespace hdmap