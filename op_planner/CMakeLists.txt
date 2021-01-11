cmake_minimum_required(VERSION 2.8.3)
project(op_planner)

find_package(catkin REQUIRED COMPONENTS
		autoware_build_flags
		cmake_modules
        op_utility
        roscpp
        # gflags
)

# find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  op_planner
   CATKIN_DEPENDS op_utility
   DEPENDS TinyXML
)

###########
## Build ##
###########

include_directories(
		include
		${catkin_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${gflags_INCLUDE_DIRS}
        ${glog_INCLUDE_DIRS}
)

set(PLANNERH_SRC
		include/op_planner/MatrixOperations.cpp
		include/op_planner/PlanningHelpers.cpp				
		include/op_planner/OpendriveMapRoad.cpp
		include/op_planner/OpendriveMapLoader.cpp
		include/op_planner/xml_helpers.cpp
		include/op_planner/RoadNetwork.cpp
		include/op_planner/Spiral.cpp
		include/op_planner/PathTimeGraph.cpp
		include/op_planner/PolynomialCurve.cpp
        include/op_planner/CartesianFrenetConversion.cpp
        include/op_planner/EndConditionSampler.cpp
        include/op_planner/FeasibleRegion.cpp
        include/op_planner/PredictionQuerier.cpp
        include/op_planner/ConstraintChecker.cpp
        include/op_planner/CollisionChecker.cpp
        include/op_planner/box2d.cpp
        include/op_planner/planning_gflags.cpp
        include/op_planner/RoadElement.cpp
        include/op_planner/vec2d.cpp
        include/op_planner/log.h
        include/op_planner/time.h
        include/op_planner/trajectory_stitcher.cpp
        include/op_planner/macro.h

        include/op_planner/reference_line/reference_line_base.cpp
        include/op_planner/reference_line/st_point.cpp
        include/op_planner/reference_line/st_boundary.cpp
        include/op_planner/reference_line/path_obstacle.cpp
        include/op_planner/reference_line/path_decision.cpp
        include/op_planner/reference_line/reference_line_info.cpp
        include/op_planner/reference_line/frame.cpp

        include/op_planner/traffic_rule/change_lane.cpp
        include/op_planner/traffic_rule/traffic_decider.cpp
        include/op_planner/traffic_rule/util.cpp
        include/op_planner/traffic_rule/destination.cpp
        include/op_planner/traffic_rule/crosswalk.cpp
        include/op_planner/traffic_rule/signal_light.cpp
        
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	    ${PLANNERH_SRC}
)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		# ${OpenCV_LIBS}
        ${TinyXML_LIBRARIES}
        gflags
        glog
)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		)


install(TARGETS op_planner
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

        SET(CMAKE_BUILD_TYPE Debug)
