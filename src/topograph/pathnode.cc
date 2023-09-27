#include "topograph.h"

namespace hdmap {

inline auto find_closest_point_on_curve = [](auto curve, double x, double y, double start_s, double end_s) {
    const double TOLERANCE           = 1e-8; // 精度要求
    auto         closest_s           = 0.5 * (start_s + end_s);
    auto         min_distance_square = double{1e6};

    while (end_s - start_s > TOLERANCE) {
        double mid_s        = 0.5 * (start_s + end_s);
        auto [cur_x, cur_y] = curve(mid_s);

        double cur_distance_square = distance_square(x, y, cur_x, cur_y); // 计算当前点与目标点间的距离的平方

        if (cur_distance_square <= min_distance_square) { // 如果当前点更接近
            closest_s           = mid_s;
            min_distance_square = cur_distance_square;
        }

        auto [xr, yr] = curve(mid_s + 0.1);
        auto r_dis    = distance_square(x, y, xr, yr);
        if (cur_distance_square < r_dis) // 判断接下来的查找区间应该在左侧还是右侧
            end_s = mid_s;
        else
            start_s = mid_s;
    }
    std::cout << closest_s << min_distance_square << std::endl;
    return make_tuple(closest_s, sqrt(min_distance_square));
};

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
    return planner::find_closest_point_on_curve(curv, point.x, point.y, start_s_, end_s_);
}

} // namespace hdmap