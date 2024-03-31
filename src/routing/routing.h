#ifndef __METHOD__
#define __METHOD__

#include "../topograph/topograph.h"

namespace hdmap {

class Routing {
  public:
    enum METHOD {
        DIJKSTRA = 1,
        ASTAR    = 2,
    };

    std::shared_ptr<hdmap::TopoGraph> topo_;

  private:
    int                         method_;
    std::shared_ptr<PathNode>   cur_node_;
    bool                        searched_;
    vector<hdmap::Lane::LaneID> result_;

  public:
    Routing(int method, std::shared_ptr<hdmap::TopoGraph> topo) : method_(method), topo_(topo) {
        searched_ = Search();
        if (searched_) {
            if (method_ == METHOD::DIJKSTRA)
                result_ = TraceBack();
            else if (method_ == METHOD::ASTAR)
                result_ = TraceBack();
            for (auto& node_id : result_) std::cout << "node: " << node_id.Str() << std::endl;
        } else
            std::cerr << "No route!" << std::endl;
    }

    auto Search() -> bool {
        if (method_ == METHOD::DIJKSTRA)
            return Dijkstra();
        else if (method_ == METHOD::ASTAR)
            return Astar();
        else
            return Dijkstra();
    }

    auto Ready() -> bool {
        return searched_;
    }

    /* get result */
    auto TraceBack() -> vector<hdmap::Lane::LaneID> {
        auto node_seq = vector<hdmap::Lane::LaneID>{};
        node_seq.emplace_back(topo_->end_node_->lane_full_id());
        while (topo_->nodes_.at(node_seq[0])->father_ != nullptr) {
            node_seq.emplace(node_seq.begin(), topo_->nodes_.at(node_seq[0])->father_->lane_full_id());
        }
        return node_seq;
    }

    auto GlobalRefTraj() -> Trajectory {
        auto ref_traj = Trajectory{};
        for (int i = 0; i < result_.size(); i++) {
            auto lane_full_id = result_[i];
            auto cur_traj =
                topo_->origin_map_->roads_.at(lane_full_id.road_id_).ref_trajs_.at(lane_full_id.traj_id_);
            if (i == 0)
                ref_traj.Update(cur_traj.segments_);
            else if (lane_full_id.road_id_ != result_[i - 1].road_id_) {
                /* TODO 两个traj未连接，用clothoid连接，这是不对的 */
                if (not ref_traj.segments_.empty() and cur_traj.GetPoint(0) != ref_traj.GetPoint(ref_traj.length_))
                    ref_traj.Update(Segment(hdmap::Segment::SEGMENT_TYPE::CLOTHOID, ref_traj.GetPoint(ref_traj.length_),
                                            cur_traj.GetPoint(0)));
                ref_traj.Update(cur_traj.segments_);
            } else
                continue;
        }
        return ref_traj;
    }

    auto Result() -> vector<hdmap::Lane::LaneID> {
        return result_;
    }

    auto AtLane(int road_id, int traj_id, double s) -> hdmap::Lane::LaneID {
        for (auto lane_full_id : result_) {
            auto lane = topo_->origin_map_->lanes_.at(lane_full_id);
            if (lane_full_id.road_id_ == road_id and lane.start_s_ < s and lane.end_s_ > s)
                return lane.lane_full_id();
        }
        return hdmap::Lane::LaneID{-1, -1, -1, -1, -1, -1};
    }

    auto RoadSeq() -> vector<int> {
        auto seq = vector<int>{};
        for (size_t i = 0; i < result_.size(); ++i)
            if (i == 0 or result_[i].road_id_ != result_[i - 1].road_id_) seq.emplace_back(result_[i].road_id_);
        return seq;
    }

    /* methods */
    auto Dijkstra() -> bool;
    auto Astar() -> bool;
};

} // namespace hdmap

#endif // __METHOD__