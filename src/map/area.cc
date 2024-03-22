#include "map.h"

namespace hdmap {

BaseArea::BaseArea(int id, std::shared_ptr<Map> map) : id_(id), map_(map) {}

auto BaseArea::Reference(int traj_id, vector<Segment> segments) -> void {
    if (not ref_trajs_.count(traj_id)) ref_trajs_.insert_or_assign(traj_id, Trajectory{});
    ref_trajs_.at(traj_id).Update(segments);
}

auto BaseArea::Reference(int traj_id, Segment segment) -> void {
    if (not ref_trajs_.count(traj_id)) ref_trajs_.insert_or_assign(traj_id, Trajectory{});
    ref_trajs_.at(traj_id).Update(segment);
}

Road::Road(int id, std::shared_ptr<Map> map) : BaseArea(id, map) {
    type_ = BaseArea::AreaType::Road;
}

Boundary::Boundary(int id, std::shared_ptr<Map> map) : BaseArea(id, map) {}

Junction::Junction(int id, std::shared_ptr<Map> map) : BaseArea(id, map) {}

Unstructured::Unstructured(int id, std::shared_ptr<Map> map) : BaseArea(id, map) {}

auto Unstructured::AddEntrance(Point entrance) {
    entrances_.emplace_back(entrance);
}

auto Unstructured::AddExit(Point exit_point) {
    exits_.emplace_back(exit_point);
}

auto Unstructured::AddObstacle(int id, Segment segment) {
    obstacles_.insert_or_assign(id, segment);
}

} // namespace hdmap
