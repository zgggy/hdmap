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
        node_seq.emplace_back(topo_->end_node_->id_);
        while (topo_->nodes_[node_seq[0]]->father_ != nullptr) {
            node_seq.emplace(node_seq.begin(), topo_->nodes_[node_seq[0]]->father_->id_);
        }
        return node_seq;
    }

    auto GlobalRefTraj() -> Trajectory {
        auto ref_traj = Trajectory{};
        for (int i = 0; i < result_.size(); i++) {
            auto id       = result_[i];
            auto cur_traj = topo_->nodes_[id]->road_->ref_traj_;
            if (i == 0)
                ref_traj.Update(cur_traj.segments_);
            else if (id.road != result_[i - 1].road) {
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

    auto AtLane(int road, double s) -> hdmap::Lane::LaneID {
        for (auto lane_id : result_) {
            auto lane = topo_->origin_map_->lanes_[lane_id];
            if (lane->id_.road == road and lane->start_s_ < s and lane->end_s_ > s) return lane->id_;
        }
        return hdmap::Lane::LaneID{-1, -1, -1, -1, -1};
    }

    auto RoadSeq() -> vector<int> {
        auto seq = vector<int>{};
        for (size_t i = 0; i < result_.size(); ++i)
            if (i == 0 or result_[i].road != result_[i - 1].road) seq.emplace_back(result_[i].road);
        return seq;
    }

    /* methods */
    auto Dijkstra() -> bool;
    auto Astar() -> bool;
};

} // namespace hdmap

#endif // __METHOD__