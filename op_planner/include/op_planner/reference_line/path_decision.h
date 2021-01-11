#pragma once

#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include "op_planner/reference_line/path_obstacle.h"
#include "op_planner/planning_gflags.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"

namespace PlannerHNS
{

    /**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
    class PathDecision
    {
    public:
        PathDecision() = default;

        PathObstacle *AddPathObstacle(const PathObstacle &path_obstacle);

        const IndexedList<std::string, PathObstacle> &path_obstacles() const;

        bool AddLateralDecision(const std::string &tag, const std::string &object_id,
                                const ObjectDecision &decision);
        bool AddLongitudinalDecision(const std::string &tag,
                                     const std::string &object_id,
                                     const ObjectDecision &decision);

        const PathObstacle *Find(const std::string &object_id) const;

        PathObstacle *Find(const std::string &object_id);

        void SetStBoundary(const std::string &id, const StBoundary &boundary);
        void EraseStBoundaries();
        ObjectDecision main_stop() const { return main_stop_; }
        double stop_reference_line_s() const { return stop_reference_line_s_; }
        bool MergeWithMainStop(const ObjectDecision &obj_stop, const std::string &obj_id,
                               const ReferenceLine &ref_line,
                               const SLBoundary &adc_sl_boundary);

    private:
        IndexedList<std::string, PathObstacle> path_obstacles_;
        ObjectDecision main_stop_;
        double stop_reference_line_s_ = std::numeric_limits<double>::max();
    };

} // namespace PlannerHNS
