#ifndef __NEAREST__
#define __NEAREST__

#include <algorithm>
#include <cmath>
#include <tuple>

#include "vec2d.h"

using std::min, std::max;
using std::tuple, std::make_tuple;
namespace planner {

/** 曲线函数要用lambda函数，示例：
 * auto curve = [](double s) {
 *     return make_pair(fun1(s), fun2(s));
 * };
 */

static auto distance_square(double x1, double y1, double x2, double y2) -> double {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return dx * dx + dy * dy;
}

// 在[start_s, end_s]范围内，寻找曲线上最接近(x, y)的点的参数s
static auto find_closest_point_on_curve = [](auto curve, double x, double y, double start_s, double end_s,
                                             double jump_s = 1) {
    // double nearest_s = MAXFLOAT;
    // for (double s = start_s; s <= end_s; s += jump_s) {
    //     auto [cur_x, cur_y] = curve(s);
    //     auto cur_dis        = distance_square(x, y, cur_x, cur_y);
    //     if (cur_dis < nearest_s) { nearest_s = cur_dis; }
    // }
    // start_s = (nearest_s - jump_s) < start_s ? start_s : (nearest_s - jump_s);
    // end_s   = (nearest_s + jump_s) > end_s ? end_s : (nearest_s + jump_s);

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


auto find_closest_point_id_on_discrete_traj(const auto& traj, double x, double y, int jump_num = 10) {
    auto closest_idx         = int{-1};
    auto min_distance_square = double{1e6};
    for (int cur_idx = 0; cur_idx < traj.size(); cur_idx += jump_num) {
        auto cur_distance_square = distance_square(traj[cur_idx].x, traj[cur_idx].y, x, y);
        if (cur_distance_square < min_distance_square) {
            closest_idx         = cur_idx;
            min_distance_square = cur_distance_square;
        }
    }
    int traj_point_num = traj.size();
    int min_index      = max(closest_idx - jump_num, 0);
    int max_index      = min(closest_idx + jump_num, traj_point_num - 1);
    for (int cur_idx = min_index; cur_idx <= max_index; cur_idx++) {
        auto cur_distance_square = distance_square(traj[cur_idx].x, traj[cur_idx].y, x, y);
        if (cur_distance_square < min_distance_square) {
            closest_idx         = cur_idx;
            min_distance_square = cur_distance_square;
        }
    }
    return make_tuple(closest_idx, min_distance_square);
};

auto can_be_project(const auto& traj, int nearest_id, double x, double y) {
    Vec2D vector_match_point(traj[nearest_id].x, traj[nearest_id].y); // 最近点位矢
    Vec2D vector_match_point_direction(cos(traj[nearest_id].theta_),
                                       sin(traj[nearest_id].theta_)); // 最近点方向向量
    Vec2D vector_r(x, y);                                             // 待投影点的位矢

    Vec2D vector_d = vector_r - vector_match_point; // 待投影点到最近点之间的向量
    double ds = vector_match_point_direction * vector_d; // 将vector_d往最近点方向向量上投影，得到两点近似s的差
    // 注意：ds为矢量，具有方向性，ds>0表示待投影点在最近点的前方，ds<0表示待投影点在最近点的后方

    auto s = traj[nearest_id].s + ds; // 计算待投影点的s值

    if (s > traj.back().s || s < 0) { // 若发现投影点的s已不在这条参考线上，则无法实现投影，直接返回
        return false;
    }
    return true;
}

auto find_nearest_and_check(const auto& traj, double x, double y, int jump_num) -> tuple<bool, int> {
    auto [nearest_id, min_distance_square] = find_closest_point_id_on_discrete_traj(traj, x, y, jump_num);
    bool is_correct                        = can_be_project(traj, nearest_id, x, y);
    return make_tuple(is_correct, nearest_id);
}

} // namespace planner

#endif // __NEAREST__