#include "map.h"

namespace hdmap {

Road::Road(int id, std::shared_ptr<Map> map) : id_(id), map_(map) {}

auto Road::Reference(vector<Segment> segments) -> void {
    ref_traj_.Update(segments);
    sections_ = {std::make_pair(0, ref_traj_.Length())};
}

auto Road::Reference(Segment segment) -> void {
    ref_traj_.Update(segment);
    sections_ = {std::make_pair(0, ref_traj_.Length())};
}

auto Road::Cut(std::initializer_list<double> sec_list) -> void {
    sections_.clear();
    auto segs = vector<double>{sec_list};
    sections_.emplace_back(std::make_pair(0, segs[0]));
    for (int i = 0; i != segs.size() - 1; ++i) {
        sections_.emplace_back(std::make_pair(segs[i], segs[i + 1]));
    }
    std::cout << "CUT length: " << ref_traj_.length_ << std::endl;
    sections_.emplace_back(std::make_pair(segs.back(), ref_traj_.length_));
}

} // namespace hdmap
