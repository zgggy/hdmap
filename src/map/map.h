#ifndef __MAP_ELEMENTS__
#define __MAP_ELEMENTS__


#include <clothoids_wrapper/clothoids_main/Clothoids.hh>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

// #include "../../clothoids_wrapper/include/clothoids_wrapper/clothoids_main/Clothoids.hh"
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
    };
    int                  type_;
    Point                start_;
    Point                end_;
    double               length_;
    G2lib::G2solve3arc   g2_;
    G2lib::ClothoidCurve spiral_;

  public:
    Segment(int type, Point start, Point end);
    Segment(Point start, double dk, double s);
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
    vector<Segment> segments_;
    double          length_ = 0;

  public:
    auto Update(vector<Segment> segments) -> void;
    auto Update(Segment segment) -> void;
    auto Cat(const Trajectory& trajectory) -> void;
    auto Length() -> double;
    auto GetValue(SAMPLE_TYPE sample_type, double s) -> double;
    auto GetPoint(double s) -> Point;
    auto Sample(SAMPLE_TYPE sample_type, double waypoint_interval) -> vector<double>;
    auto SampleAll(double waypoint_interval) -> tuple<vector<double>, vector<double>, vector<double>, vector<double>>;
    auto SampleTraj(double waypoint_interval) -> vector<Point>;
    auto NearestWith(SimpPoint point) -> tuple<double, double>;
};

class BaseLane : public std::enable_shared_from_this<BaseLane> {
    friend class Map;

  public:
    enum PARAMETER_TYPE {
        SE = 1,
        AB = 2,
        F  = 3,
    };

  public:
    std::shared_ptr<Map>  map_;
    std::shared_ptr<Road> road_;
    double                start_s_, end_s_;
    PolyPara              parameters_;

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
    friend class Map;

  public:
    struct LaneID {
        int  road, section, group, lane, unit;
        auto Str() -> std::string;
        bool operator<(const LaneID& other) const;
        bool operator>(const LaneID& other) const;
        bool operator==(const LaneID& other) const;
        bool operator!=(const LaneID& other) const;
    };
    LaneID id_;

  public:
    Lane() = default;
    Lane(std::shared_ptr<Map> map, std::shared_ptr<Road> road, int section, int group, int lane, PolyPara parameters,
         int parameter_type);
    auto Cut(std::initializer_list<double> unit_list) -> vector<Lane>;
};

class Solid : public BaseLane {
  public:
    struct SolidID {
        int    road, section;
        double mid_group;
        auto   Str() -> std::string;
        bool   operator<(const SolidID& other) const;
        bool   operator>(const SolidID& other) const;
        bool   operator==(const SolidID& other) const;
        bool   operator!=(const SolidID& other) const;
    };
    SolidID id_;

  public:
    Solid(std::shared_ptr<Map> map, std::shared_ptr<Road> road, int section, double mid_group, PolyPara parameters,
          int parameter_type);
};

class Road : public std::enable_shared_from_this<Road> {
    friend class Lane;
    enum RoadAttribute {
        Normal   = 0u,
        Junction = 1u,
    };

  public:
    std::shared_ptr<Map>              map_;
    int                               id_;
    Trajectory                        ref_traj_;
    vector<std::pair<double, double>> sections_;
    RoadAttribute                     attribute_;

  public:
    Road() = default;
    Road(int id, std::shared_ptr<Map> map);
    auto Reference(vector<Segment> segments) -> void;
    auto Reference(Segment segment) -> void;
    auto Cut(std::initializer_list<double> sec_list) -> void;
};

class Boundary : public std::enable_shared_from_this<Boundary> {
  public:
    std::shared_ptr<Map> map_;
    int                  id_;
    Trajectory           ref_traj_;

  public:
    Boundary(int id, std::shared_ptr<Map> map);
    auto Reference(vector<Segment> segments) -> void;
    auto Reference(Segment segment) -> void;
};


class Map : public std::enable_shared_from_this<Map> {
  public:
    std::map<int, std::shared_ptr<Road>>             roads_;
    std::map<int, std::shared_ptr<Boundary>>         boundaries_;
    std::map<Lane::LaneID, std::shared_ptr<Lane>>    lanes_;
    std::map<Solid::SolidID, std::shared_ptr<Solid>> solids_;

  public:
    Map() = default;
    void AddRoad(int road_id);
    void AddLane(int road_id, int section_num, int group_num, std::initializer_list<double> parameters, int param_type);
    void AddSolid(int road_id, int section_num, double mid_group, std::initializer_list<double> parameters,
                  int param_type);
    auto AtRoad(SimpPoint point) -> tuple<int, double>;
    auto AtRoadPtr(SimpPoint point) -> std::shared_ptr<Road>;
    auto AtRoutingRoad(SimpPoint point, vector<int> road_id_set) -> tuple<int, double>;
    auto AtLane(SimpPoint point) -> tuple<Lane::LaneID, double>;
    auto AtLanePtr(SimpPoint point) -> std::shared_ptr<Lane>;
    auto AtLanesByS(int road_id, double s) -> vector<std::shared_ptr<Lane>>;
};

} // namespace hdmap

#endif // __MAP_ELEMENTS__