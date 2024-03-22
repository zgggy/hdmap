#ifndef __HDMAP_NEAREST__
#define __HDMAP_NEAREST__

#include <algorithm>
#include <cmath>
#include <tuple>

using std::min, std::max;
using std::tuple, std::make_tuple;
namespace hdmap {

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
static auto find_closest_point_on_curve = [](auto curve, double x, double y, double start_s, double end_s) {
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
    return make_tuple(closest_s, sqrt(min_distance_square));
};

} // namespace hdmap

#endif // __HDMAP_NEAREST__