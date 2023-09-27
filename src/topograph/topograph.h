#ifndef __TOPO_ELEMENTS__
#define __TOPO_ELEMENTS__

#include "../map/map.h"
#include "../method/nearest.h"

namespace hdmap {

using planner::Point;

class PathNode : public Lane {
    friend class DynamicNodeDijkstra;
    friend class TopoGraph;
    friend class Routing;

  private:
    std::shared_ptr<PathNode> father_;
    double                    global_start_s_;
    double                    global_end_s_;
    double                    father_end_s_;
    double                    cost_;
    bool                      closed_;
    bool                      is_best_;
    bool                      side_to_best_;

  public:
    vector<LaneID> predecessors_, successors_, sidecessors_;

  public:
    PathNode(std::shared_ptr<Lane> lane);
    auto NearestWith(Point point) -> tuple<double, double>;
};

class TopoGraph {
  public:
    std::shared_ptr<hdmap::Map> origin_map_;

    std::map<Lane::LaneID, std::shared_ptr<PathNode>> nodes_;
    std::shared_ptr<PathNode>                         start_node_;
    std::shared_ptr<PathNode>                         end_node_;

  public:
    TopoGraph(std::shared_ptr<hdmap::Map> origin_map);
    void SetStartAndEndNode(Lane::LaneID start_id, double start_s, Lane::LaneID end_id, double end_s);
    void UpdateTopoGraph();
    void CutGroup(int road_id, int section_num, int group, std::initializer_list<double> unit_list);
    void Print();
    // auto GetRefTraj(Lane::LaneID lane_id, double s, double length, double waypoint_interval) -> vector<Point>;
    auto AtNode(int at_road, planner::Point point) -> tuple<Lane::LaneID, double>;
    auto AtNode(Lane::LaneID last_at_node, planner::Point point) -> tuple<Lane::LaneID, double>;
};

} // namespace hdmap

#endif // __TOPO_ELEMENTS__