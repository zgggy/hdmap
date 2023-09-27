#include "routing.h"

namespace hdmap {

auto Routing::Dijkstra() -> bool {
    auto fail_times           = int{0};
    cur_node_                 = topo_->start_node_;
    auto last_cur_node        = cur_node_;
    topo_->start_node_->cost_ = topo_->start_node_->RoadLength();
    while (true) {
        last_cur_node = cur_node_;
        auto min_cost = MAXFLOAT;
        for (auto node_item : topo_->nodes_) {
            auto node = node_item.second;
            if (not node->closed_ and node->cost_ < min_cost) {
                cur_node_ = node;
                min_cost  = node->cost_;
            }
        }
        if (cur_node_->closed_ and cur_node_ == last_cur_node) {
            ++fail_times;
        } else
            fail_times = 0;
        if (fail_times > 10) return false;
        cur_node_->closed_ = true;
        if (topo_->end_node_->closed_) return true;
        for (auto node_id : cur_node_->successors_) {
            std::cout<< cur_node_->id_.Str() << " -> " << node_id.Str() << std::endl;
            auto node = topo_->nodes_[node_id];
            std::cout<< "  cur_node cost: " << cur_node_->cost_ << std::endl;
            if (not node->closed_) {
                auto new_cost = cur_node_->cost_ + node->RoadLength();
                std::cout<< "  new cost: " << new_cost << ", node cost: " << node->cost_ << std::endl;
                if (node->cost_ > new_cost) {
                    node->father_ = cur_node_;
                    node->cost_   = new_cost;
                }
            }
        }
        for (auto node_id : cur_node_->sidecessors_) {
            std::cout<< cur_node_->id_.Str() << " -> " << node_id.Str() << std::endl;
            auto node = topo_->nodes_[node_id];
            std::cout<< "  cur_node cost: " << cur_node_->cost_ << std::endl;
            if (not node->closed_) {
                auto new_cost = cur_node_->cost_ + 1;
                std::cout<< "  new cost: " << new_cost << ", node cost: " << node->cost_ << std::endl;
                if (node->cost_ > new_cost) {
                    node->father_ = cur_node_;
                    node->cost_   = new_cost;
                }
            }
        }
    }
}

} // namespace hdmap