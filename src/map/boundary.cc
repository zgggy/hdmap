#include "map.h"

namespace hdmap {

Boundary::Boundary(int id, std::shared_ptr<Map> map) : id_(id), map_(map) {}

auto Boundary::Reference(vector<Segment> segments) -> void {
    ref_traj_.Update(segments);
}

auto Boundary::Reference(Segment segment) -> void {
    ref_traj_.Update(segment);
}

} // namespace hdmap
