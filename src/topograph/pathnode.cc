#include "topograph.h"

namespace hdmap {

PathNode::PathNode(std::shared_ptr<Lane> lane) {
    map_          = lane->map_;
    road_         = lane->road_;
    start_s_      = lane->start_s_;
    end_s_        = lane->end_s_;
    parameters_   = lane->parameters_;
    id_           = lane->id_;
    father_       = nullptr;
    cost_         = MAXFLOAT;
    closed_       = false;
    is_best_      = false;
    side_to_best_ = false;
    predecessors_.clear();
    successors_.clear();
    sidecessors_.clear();
    std::cout << "start: " << GetPoint(start_s_).Str() << std::endl;
    std::cout << "end  : " << GetPoint(end_s_).Str() << std::endl;
    std::cout << start_s_ << " " << end_s_ << std::endl;
    auto [a, b] = NearestWith(GetPoint(start_s_));
    std::cout << "near : " << GetPoint(a).Str() << std::endl;
    std::cout << start_s_ << "," << end_s_ << "," << a << "," << b << std::endl;
}

auto PathNode::NearestWith(Point point) -> tuple<double, double> {
    auto curv = [this](double s) {
        auto p = GetPoint(s);
        return make_tuple(p.x, p.y);
    };
    return find_closest_point_on_curve(curv, point.x, point.y, start_s_, end_s_);
}

} // namespace hdmap