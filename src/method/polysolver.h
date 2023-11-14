#ifndef __HDMAP_POLYSOVER__
#define __HDMAP_POLYSOVER__

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

using std::tuple, std::make_tuple;
using std::vector;
using PolyPara = tuple<double, double, double, double, double, double>;

namespace hdmap {
static auto solve_poly_5(double start_s, double start_l, double start_dl, double start_ddl, double end_s, double end_l,
                         double end_dl, double end_ddl) -> PolyPara {
    Eigen::MatrixXd A(6, 6);
    Eigen::MatrixXd b(6, 1);
    A << pow(start_s, 5), pow(start_s, 4), pow(start_s, 3), pow(start_s, 2), start_s, 1,  //
        5 * pow(start_s, 4), 4 * pow(start_s, 3), 3 * pow(start_s, 2), 2 * start_s, 1, 0, //
        20 * pow(start_s, 3), 12 * pow(start_s, 2), 6 * pow(start_s, 1), 2, 0, 0,         //
        pow(end_s, 5), pow(end_s, 4), pow(end_s, 3), pow(end_s, 2), end_s, 1,             //
        5 * pow(end_s, 4), 4 * pow(end_s, 3), 3 * pow(end_s, 2), 2 * end_s, 1, 0,         //
        20 * pow(end_s, 3), 12 * pow(end_s, 2), 6 * pow(end_s, 1), 2, 0, 0;               //
    b << start_l, start_dl, start_ddl, end_l, end_dl, end_ddl;
    auto a = A.inverse() * b;
    return make_tuple(a(0, 0), a(1, 0), a(2, 0), a(3, 0), a(4, 0), a(5, 0));
}

static auto solve_poly_5(double s0, double s1, PolyPara l_dl_ddl) {
    auto [l0, dl0, ddl0, l1, dl1, ddl1] = l_dl_ddl;
    return solve_poly_5(s0, l0, dl0, ddl0, s1, l1, dl1, ddl1);
}

static auto poly_5(PolyPara parameters, double s, int order) -> double {
    auto [a, b, c, d, e, f] = parameters;
    auto [s2, s3, s4, s5]   = make_tuple(pow(s, 2), pow(s, 3), pow(s, 4), pow(s, 5));

    switch (order) {
        case 0:
            return a * s5 + b * s4 + c * s3 + d * s2 + e * s + f;
        case 1:
            return 5 * a * s4 + 4 * b * s3 + 3 * c * s2 + 2 * d * s + e;
        case 2:
            return 20 * a * s3 + 12 * b * s2 + 6 * c * s + 2 * d;
        case 3:
            return 60 * a * s2 + 24 * b * s + 6 * c;
        case 4:
            return 120 * a * s + 24 * b;
        case 5:
            return 120 * a;
        default:
            std::cerr << "wrong order！" << std::endl;
            return 0.0; // 阶数不在0至5范围内，返回默认值
    }
}
} // namespace hdmap
#endif // __HDMAP_POLYSOVER__