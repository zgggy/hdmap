# HDMAP

![hdmap](doc/img/hdmap_128.png)

## CMakeLists

to use this lib, write at least these 3 statements:  

```CMake
add_compile_options(
    -std=c++17 # for new future of c++
    -fconcepts # for auto paremeters in lambda
)

include(hdmap/CMakeLists.include)

target_link_libraries(${PROJECT_NAME}
    # other libs
    hdmap_lib # required!!!
)
```  

a sample:  

```CMake
cmake_minimum_required(VERSION 3.0.2)
project(node)

add_compile_options(
    -std=c++17 # for new future of c++
    -fconcepts # for auto paremeters in lambda
)

find_package(catkin REQUIRED COMPONENTS)

include(hdmap/CMakeLists.include)

add_executable(node
    main.cc
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    hdmap_lib
)
```

## C++

just  `#include <hdmap.h>` and instantiate them by `shared_ptr`(suggested), a sample:  

```cpp
#include <hdmap.h>

#include <memory>

using hdmap::Map, hdmap::Segment, hdmap::BaseLane, hdmap::Lane;

int main() {
    auto map = std::make_shared<Map>();
    map->AddRoad(0);
    map->roads_[0]->Reference(
        Segment(Segment::SEGMENT_TYPE::CLOTHOID, planner::Point{0, 0, 0, 0}, planner::Point{2000, 0, 0, 0}));
    map->AddLane(0, 0, 0, {0}, BaseLane::PARAMETER_TYPE::F);

    auto topo = std::make_shared<hdmap::TopoGraph>(map);
    topo->SetStartAndEndNode(Lane::LaneID{0, 1, 1, 0}, 9, Lane::LaneID{0, 2, 0, 2}, 19);
    topo->UpdateTopoGraph();
    topo->Print();
    auto navi        = std::make_shared<hdmap::Routing>(hdmap::Routing::METHOD::DIJKSTRA, topo);
    auto traj        = navi->GlobalRefTraj();
    auto sample_traj = traj.SampleTraj(0.1);
    for (auto sample : sample_traj) std::cout << sample.x << " " << sample.y << std::endl;
}
```
