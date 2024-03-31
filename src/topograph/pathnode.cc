#include "topograph.h"

namespace hdmap {

PathNode::PathNode(const Lane& lane) : Lane(lane) {
    std::cout << "construct lane id: " << lane_full_id().Str() << std::endl;
    father_       = nullptr;
    cost_         = MAXFLOAT;
    closed_       = false;
    is_best_      = false;
    side_to_best_ = false;
    predecessors_.clear();
    successors_.clear();
    sidecessors_.clear();
    std::cout << "start s: " << start_s_ << " " << GetPoint(start_s_).Str() << std::endl;
    std::cout << "end s: " << end_s_ << " " << GetPoint(end_s_).Str() << std::endl;
}

auto PathNode::NearestWith(Point point) -> tuple<double, double> {
    auto curv = [this](double s) {
        auto p = GetPoint(s);
        return make_tuple(p.x, p.y);
    };
    return find_closest_point_on_curve(curv, point.x, point.y, start_s_, end_s_);
}

} // namespace hdmap