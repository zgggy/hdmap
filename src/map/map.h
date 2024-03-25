#ifndef __MAP_ELEMENTS__
#define __MAP_ELEMENTS__


#include <clothoids_wrapper/clothoids_main/Clothoids.hh>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "../method/nearest.h"
#include "../method/polysolver.h"
#include "../method/vec2d.h"

namespace hdmap {

class Segment;
class Trajectory;
class Lane;
class Solid;
class Road;
class Map;

enum SAMPLE_TYPE {
    X = 1,
    Y = 2,
    T = 3,
    C = 4,
};

class Segment {
  public:
    enum SEGMENT_TYPE {
        LINE     = 1,
        CLOTHOID = 2,
        SPIRAL   = 3,
    };
    SEGMENT_TYPE         type_;
    Point                start_;
    Point                end_;
    double               length_;
    G2lib::G2solve3arc   g2_;
    G2lib::ClothoidCurve spiral_;

  public:
    Segment(SEGMENT_TYPE type, Point start, Point end);
    Segment(Point start, double dk, double s);
    auto To3Segment() -> std::vector<Segment>;
    auto Length() -> double;
    auto GetValue(SAMPLE_TYPE sample_type, double s) -> double;
    auto GetPoint(double s) -> Point;
    auto Sample(SAMPLE_TYPE sample_type, double waypoint_interval) -> vector<double>;
    auto SampleAll(double waypoint_interval) -> tuple<vector<double>, vector<double>, vector<double>, vector<double>>;
    auto SampleTraj(double waypoint_interval) -> vector<Point>;
};

class Trajectory {
    friend class Road;

  public:
    vector<Segment>                        segments_;
    double                                 length_ = 0;
    std::vector<std::pair<double, double>> sections_;

  public:
    auto Update(vector<Segment> segments) -> void;
    auto Update(Segment segment) -> void;
    auto Cat(const Trajectory& trajectory) -> void;
    auto Cut(std::initializer_list<double> sec_list) -> void;
    auto Length() -> double;
    auto GetValue(SAMPLE_TYPE sample_type, double s) -> double;
    auto GetPoint(double s) -> Point;
    auto Sample(SAMPLE_TYPE sample_type, double waypoint_interval) -> vector<double>;
    auto SampleAll(double waypoint_interval) -> tuple<vector<double>, vector<double>, vector<double>, vector<double>>;
    auto SampleTraj(double waypoint_interval) -> vector<Point>;
    auto NearestWith(SimpPoint point) -> tuple<double, double>;
};

class BaseArea {
    friend class BaseLane;

  public:
    enum AreaType : uint8_t {
        Road         = 0u,
        Junction     = 1u,
        Boundary     = 2u,
        Unstructured = 3u,
    };

  public:
    std::shared_ptr<Map>                map_;
    int                                 id_;
    std::unordered_map<int, Trajectory> ref_trajs_;
    AreaType                            type_;

  public:
    BaseArea(int id, std::shared_ptr<Map> map);
    auto Reference(int traj_id, vector<Segment> segments) -> void;
    auto Reference(int traj_id, Segment segment) -> void;
};

class Road : public BaseArea {
  public:
  public:
    Road(int id, std::shared_ptr<Map> map);
};

class Boundary : public BaseArea {
  public:
    Boundary(int id, std::shared_ptr<Map> map);
};

class Junction : public BaseArea {
  public:
    Junction(int id, std::shared_ptr<Map> map);
};

class Unstructured : public BaseArea {
  public:
    std::vector<Point>               entrances_;
    std::vector<Point>               exits_;
    std::unordered_map<int, Segment> obstacles_;

  public:
    Unstructured(int id, std::shared_ptr<Map> map);
    auto AddEntrance(Point entrance);
    auto AddExit(Point entrance);
    auto AddObstacle(int id, Segment segment);
};

class BaseLane {
    friend class Map;

  public:
    enum PARAMETER_TYPE {
        SE = 1,
        AB = 2,
        F  = 3,
    };

  public:
    std::shared_ptr<Map> map_;
    int                  road_id_;
    int                  traj_id_;
    int                  section_id_;
    double               start_s_, end_s_;
    PolyPara             parameters_;

  public:
    BaseLane(std::shared_ptr<Map> map, int road_id, int traj_id, int section_id, PolyPara parameters,
             int parameter_type);
    auto L(double s) -> double;
    auto dL(double s) -> double;
    auto ddL(double s) -> double;
    auto RoadLength() -> double;
    auto GetPoint(double s) -> Point;
    auto Start() -> Point;
    auto End() -> Point;
    auto SampleAll(double waypoint_interval) -> tuple<vector<double>, vector<double>, vector<double>, vector<double>>;
    auto SampleTraj(double waypoint_interval) -> vector<Point>;
    auto NearestWith(SimpPoint point) -> tuple<double, double>;
};

class Lane : public BaseLane {
  public:
    struct LaneID {
        int  road_id_;
        int  traj_id_;
        int  section_id_;
        int  group_id_;
        int  lane_id_;
        int  unit_id_;
        auto Str() const -> std::string {
            return std::to_string(road_id_) + std::to_string(traj_id_) + std::to_string(section_id_) +
                   std::to_string(group_id_) + std::to_string(lane_id_) + std::to_string(unit_id_);
        }
        auto Num() const -> int {
            return road_id_ * 1e6 + traj_id_ * 1e5 + section_id_ * 1e4 + group_id_ * 1e3 + lane_id_ * 1e2 + unit_id_;
        }
        bool operator<(const LaneID& other) const {
            return Num() < other.Num();
        }
        bool operator>(const LaneID& other) const {
            return not(*this < other);
        }
        bool operator==(const LaneID& other) const {
            return road_id_ == other.road_id_ and traj_id_ == other.traj_id_ and section_id_ == other.section_id_ and
                   group_id_ == other.group_id_ and lane_id_ == other.lane_id_ and unit_id_ == other.unit_id_;
        }
        bool operator!=(const LaneID& other) const {
            return not(*this == other);
        }
    };

  public:
    int group_id_;
    int lane_id_;
    int unit_id_;

  public:
    auto lane_id_complate() -> LaneID {
        return LaneID{road_id_, traj_id_, section_id_, group_id_, lane_id_, unit_id_};
    }

  public:
    Lane(std::shared_ptr<Map> map, int road_id, int traj_id, int section_id, int group_id, int lane_id,
         PolyPara parameters, int parameter_type);
    auto Cut(std::initializer_list<double> unit_list) -> vector<Lane>;
};

class Map : public std::enable_shared_from_this<Map> {
  public:
    std::map<int, Road>          roads_;
    std::map<int, Junction>      junctions_;
    std::map<int, Unstructured>  unstructureds_;
    std::map<int, Boundary>      boundaries_;
    std::map<Lane::LaneID, Lane> lanes_;

  public:
    Map() = default;
    void AddRoad(int road_id);
    void AddLane(int road_id, int traj_id, int section_id, int group_id, std::initializer_list<double> parameters,
                 int param_type);
    auto AtLane(SimpPoint point) -> tuple<Lane::LaneID, double>;
};

} // namespace hdmap

#endif // __MAP_ELEMENTS__