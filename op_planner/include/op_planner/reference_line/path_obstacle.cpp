#include "op_planner/reference_line/path_obstacle.h"

namespace PlannerHNS
{

    namespace
    {
        const double kStBoundaryDeltaS = 0.2;       // meters
        const double kStBoundarySparseDeltaS = 1.0; // meters
        const double kStBoundaryDeltaT = 0.05;      // seconds
    }                                               // namespace

    const std::string &PathObstacle::Id() const { return id_; }

    PathObstacle::PathObstacle(const DetectedObject *obstacle) : obstacle_(obstacle)
    {
        CHECK_NOTNULL(obstacle);
        id_ = obstacle_->Id();
    }

    void PathObstacle::SetPerceptionSlBoundary(const SLBoundary &sl_boundary)
    {
        perception_sl_boundary_ = sl_boundary;
    }

    double PathObstacle::MinRadiusStopDistance() const
    {
        if (min_radius_stop_distance_ > 0)
        {
            return min_radius_stop_distance_;
        }
        constexpr double stop_distance_buffer = 0.5;
        const double min_turn_radius = car_params.turning_radius;
        double lateral_diff = car_params.width / 2.0 +
                              std::max(std::fabs(perception_sl_boundary_.start_l),
                                       std::fabs(perception_sl_boundary_.end_l));
        const double kEpison = 1e-5;
        lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
        double stop_distance =
            std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                                (min_turn_radius - lateral_diff) *
                                    (min_turn_radius - lateral_diff))) +
            stop_distance_buffer;
        // stop_distance -= vehicle_param.front_edge_to_center();
        // front axis to adc front edge
        stop_distance -= ((car_params.width - car_params.wheel_base) / 2.0);
        stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
        stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
        return stop_distance;
    }

    void PathObstacle::BuildReferenceLineStBoundary(
        const ReferenceLine &reference_line, const double adc_start_s)
    {
        const double adc_width = car_params.width;

        if (!obstacle_->HasTrajectory())
        {
            std::vector<std::pair<STPoint, STPoint>> point_pairs;
            double start_s = perception_sl_boundary_.start_s;
            double end_s = perception_sl_boundary_.end_s;
            if (end_s - start_s < kStBoundaryDeltaS)
            {
                end_s = start_s + kStBoundaryDeltaS;
            }
            // ADEBUG << " start_s: " << start_s << " end_s: " << end_s;

            if (!reference_line.IsBlockRoad(obstacle_->PerceptionBoundingBox(),
                                            adc_width))
            {
                return;
            }

            ADEBUG << "static obstacle block road: " << obstacle_->Id();

            point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                                     STPoint(end_s - adc_start_s, 0.0));
            point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                                     STPoint(end_s - adc_start_s, FLAGS_st_max_t));
            reference_line_st_boundary_ = StBoundary(point_pairs);
        }
        else
        {
            if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                          &reference_line_st_boundary_))
            {
                // ADEBUG << "Found st_boundary for obstacle " << id_;
                // ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
                //        << ", max_t = " << reference_line_st_boundary_.max_t()
                //        << ", min_s = " << reference_line_st_boundary_.min_s()
                //        << ", max_s = " << reference_line_st_boundary_.max_s();
            }
            else
            {
                ADEBUG << "No st_boundary for obstacle " << id_;
            }
        }
    }

    bool PathObstacle::BuildTrajectoryStBoundary(
        const ReferenceLine &reference_line, const double adc_start_s,
        StBoundary *const st_boundary)
    {
        const auto &object_id = obstacle_->Id();
        if (!IsValidObstacle(*obstacle_))
        {
            AERROR << "Fail to build trajectory st boundary because object is not "
                      "valid. PerceptionObstacle: "
                   << object_id;
            return false;
        }
        const double object_width = obstacle_->w;
        const double object_length = obstacle_->l;
        const auto &trajectory_points = obstacle_->predTrajectory;
        if (trajectory_points.empty())
        {
            AWARN << "object " << object_id << " has no trajectory points";
            return false;
        }

        const double adc_length = car_params.length;
        const double adc_half_length = adc_length / 2.0;
        const double adc_width = car_params.width;
        math::Box2d min_box({0, 0, 0, 0}, 1.0, 1.0, 1.0);
        math::Box2d max_box({0, 0, 0, 0}, 1.0, 1.0, 1.0);
        std::vector<std::pair<STPoint, STPoint>> polygon_points;

        SLBoundary last_sl_boundary;
        int last_index = 0;

        for (int i = 1; i < trajectory_points.size(); ++i)
        {
            ADEBUG << "last_sl_boundary ";

            if (trajectory_points[i].timeCost > FLAGS_trajectory_time_length)
            {
                break;
            }

            const auto &first_point = trajectory_points[i - 1].pos;
            const auto &second_point = trajectory_points[i].pos;

            double total_length =
                object_length + first_point.DistanceTo(second_point);

            GPSPoint center((first_point.x + second_point.x / 2.0),
                            (first_point.y + second_point.y / 2.0), 0, 0);
            math::Box2d object_moving_box(center, first_point.a,
                                          total_length, object_width);
            SLBoundary object_boundary;
            // NOTICE: this method will have errors when the reference line is not
            // straight. Need double loop to cover all corner cases.
            const double distance_xy =
                trajectory_points[last_index].pos.DistanceTo(trajectory_points[i].pos);
            if (last_sl_boundary.start_l > distance_xy ||
                last_sl_boundary.end_l < -distance_xy)
            {
                continue;
            }

            const double mid_s =
                (last_sl_boundary.start_s + last_sl_boundary.end_s) / 2.0;
            const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
            const double end_s = (i == 1) ? reference_line.Length()
                                          : std::fmin(reference_line.Length(),
                                                      mid_s + 2.0 * distance_xy);

            if (!reference_line.GetSLBoundary(object_moving_box, &object_boundary))
            {
                AERROR << "failed to calculate boundary";
                return false;
            }

            // update history record
            last_sl_boundary = object_boundary;
            last_index = i;

            // skip if object is entirely on one side of reference line.
            constexpr double kSkipLDistanceFactor = 0.4;
            const double skip_l_distance =
                (object_boundary.end_s - object_boundary.start_s) *
                    kSkipLDistanceFactor +
                adc_width / 2.0;

            if (std::fmin(object_boundary.start_l, object_boundary.end_l) >
                    skip_l_distance ||
                std::fmax(object_boundary.start_l, object_boundary.end_l) <
                    -skip_l_distance)
            {
                continue;
            }

            if (object_boundary.end_s < 0)
            { // skip if behind reference line
                continue;
            }
            constexpr double kSparseMappingS = 20.0;
            const double st_boundary_delta_s =
                (std::fabs(object_boundary.start_s - adc_start_s) > kSparseMappingS)
                    ? kStBoundarySparseDeltaS
                    : kStBoundaryDeltaS;
            const double object_s_diff =
                object_boundary.end_s - object_boundary.start_s;
            if (object_s_diff < st_boundary_delta_s)
            {
                continue;
            }
            const double delta_t =
                trajectory_points[i].timeCost - trajectory_points[i - 1].timeCost;
            double low_s = std::max(object_boundary.start_s - adc_half_length, 0.0);
            bool has_low = false;
            double high_s =
                std::min(object_boundary.end_s + adc_half_length, FLAGS_st_max_s);
            bool has_high = false;
            while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high))
            {
                if (!has_low)
                {
                    auto low_ref = reference_line.GetNearestReferencePoint(low_s);
                    has_low = object_moving_box.HasOverlap(
                        {low_ref.pos, low_ref.pos.a, adc_length, adc_width});
                    low_s += st_boundary_delta_s;
                }
                if (!has_high)
                {
                    auto high_ref = reference_line.GetNearestReferencePoint(high_s);
                    has_high = object_moving_box.HasOverlap(
                        {high_ref.pos, high_ref.pos.a, adc_length, adc_width});
                    high_s -= st_boundary_delta_s;
                }
            }
            if (has_low && has_high)
            {
                low_s -= st_boundary_delta_s;
                high_s += st_boundary_delta_s;
                double low_t =
                    (trajectory_points[i - 1].timeCost +
                     std::fabs((low_s - object_boundary.start_s) / object_s_diff) *
                         delta_t);
                polygon_points.emplace_back(
                    std::make_pair(STPoint{low_s - adc_start_s, low_t},
                                   STPoint{high_s - adc_start_s, low_t}));
                double high_t =
                    (trajectory_points[i - 1].timeCost +
                     std::fabs((high_s - object_boundary.start_s) / object_s_diff) *
                         delta_t);
                if (high_t - low_t > 0.05)
                {
                    polygon_points.emplace_back(
                        std::make_pair(STPoint{low_s - adc_start_s, high_t},
                                       STPoint{high_s - adc_start_s, high_t}));
                }
            }
        }
        if (!polygon_points.empty())
        {
            std::sort(polygon_points.begin(), polygon_points.end(),
                      [](const std::pair<STPoint, STPoint> &a,
                         const std::pair<STPoint, STPoint> &b) {
                          return a.first.t() < b.first.t();
                      });
            auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                                    [](const std::pair<STPoint, STPoint> &a,
                                       const std::pair<STPoint, STPoint> &b) {
                                        return std::fabs(a.first.t() - b.first.t()) <
                                               kStBoundaryDeltaT;
                                    });
            polygon_points.erase(last, polygon_points.end());
            if (polygon_points.size() > 2)
            {
                *st_boundary = StBoundary(polygon_points);
            }
        }
        else
        {
            return false;
        }
        return true;
    }

    const StBoundary &PathObstacle::reference_line_st_boundary() const
    {
        return reference_line_st_boundary_;
    }

    const StBoundary &PathObstacle::st_boundary() const { return st_boundary_; }

    const std::vector<std::string> &PathObstacle::decider_tags() const
    {
        return decider_tags_;
    }

    const std::vector<ObjectDecision> &PathObstacle::decisions() const
    {
        return decisions_;
    }

    bool PathObstacle::IsLateralDecision(const ObjectDecision &decision)
    {
        if (decision.decision_type == ignore ||
            decision.decision_type == nudge ||
            decision.decision_type == sidepass)
            return true;
        return false;
    }

    bool PathObstacle::IsLongitudinalDecision(const ObjectDecision &decision)
    {
        if (decision.decision_type == ignore ||
            decision.decision_type == stop ||
            decision.decision_type == yield ||
            decision.decision_type == follow ||
            decision.decision_type == overtake)
            return true;
        return false;
    }

    ObjectDecision PathObstacle::MergeLongitudinalDecision(
        const ObjectDecision &lhs, const ObjectDecision &rhs)
    {
        if (lhs.decision_type == unknow)
        {
            return rhs;
        }
        if (rhs.decision_type == unknow)
        {
            return lhs;
        }
        auto lhs_val = get_longitudinal_vaule(lhs.decision_type);
        auto rhs_val = get_longitudinal_vaule(rhs.decision_type);

        if (lhs_val < rhs_val)
        {
            return rhs;
        }
        else if (lhs_val > rhs_val)
        {
            return lhs;
        }
        else
        {
            if (lhs.decision_type == ignore)
            {
                return rhs;
            }
            else if (lhs.decision_type == stop)
            {
                return lhs.distance_s < rhs.distance_s ? lhs : rhs;
            }
            else if (lhs.decision_type == yield)
            {
                return lhs.distance_s < rhs.distance_s ? lhs : rhs;
            }
            else if (lhs.decision_type == follow)
            {
                return lhs.distance_s < rhs.distance_s ? lhs : rhs;
            }
            else if (lhs.decision_type == overtake)
            {
                return lhs.distance_s < rhs.distance_s ? lhs : rhs;
            }
            else
            {
                std::cout << "error decision" << std::endl;
            }
        }
        return lhs; // stop compiler complaining
    }

    const ObjectDecision &PathObstacle::LongitudinalDecision() const
    {
        return longitudinal_decision_;
    }

    const ObjectDecision &PathObstacle::LateralDecision() const
    {
        return lateral_decision_;
    }

    bool PathObstacle::IsIgnore() const
    {
        return IsLongitudinalIgnore() && IsLateralIgnore();
    }

    bool PathObstacle::IsLongitudinalIgnore() const
    {
        return longitudinal_decision_.decision_type == ignore;
    }

    bool PathObstacle::IsLateralIgnore() const
    {
        return lateral_decision_.decision_type == ignore;
        // return lateral_decision_.has_ignore();
    }

    ObjectDecision PathObstacle::MergeLateralDecision(
        const ObjectDecision &lhs, const ObjectDecision &rhs)
    {
        if (lhs.decision_type == unknow)
        {
            return rhs;
        }
        if (rhs.decision_type == unknow)
        {
            return lhs;
        }
        const auto lhs_val = get_lateral_vaule(lhs.decision_type);
        const auto rhs_val = get_lateral_vaule(rhs.decision_type);

        if (lhs_val < rhs_val)
        {
            return rhs;
        }
        else if (lhs_val > rhs_val)
        {
            return lhs;
        }
        else
        {
            if (lhs.decision_type == ignore || lhs.decision_type == sidepass)
            {
                return rhs;
            }
            else if (lhs.decision_type == nudge)
            {
                return std::fabs(lhs.distance_l) > std::fabs(rhs.distance_l) ? lhs : rhs;
            }
        }
        DCHECK(false) << "Does not have rule to merge decision: ";
        return lhs;
    }

    bool PathObstacle::HasLateralDecision() const
    {
        return lateral_decision_.decision_type != unknow;
    }

    bool PathObstacle::HasLongitudinalDecision() const
    {
        return longitudinal_decision_.decision_type != unknow;
    }

    bool PathObstacle::HasNonIgnoreDecision() const
    {
        return (HasLateralDecision() && !IsLateralIgnore()) ||
               (HasLongitudinalDecision() && !IsLongitudinalIgnore());
    }

    const DetectedObject *PathObstacle::obstacle() const { return obstacle_; }

    void PathObstacle::AddLongitudinalDecision(const std::string &decider_tag,
                                               const ObjectDecision &decision)
    {
        // DCHECK(IsLongitudinalDecision(decision))
        //     << "Decision: " << decision.ShortDebugString()
        //     << " is not a longitudinal decision";
        longitudinal_decision_ =
            MergeLongitudinalDecision(longitudinal_decision_, decision);

        decisions_.push_back(decision);
        decider_tags_.push_back(decider_tag);
    }

    void PathObstacle::AddLateralDecision(const std::string &decider_tag,
                                          const ObjectDecision &decision)
    {
        DCHECK(IsLateralDecision(decision))
            << "Decision: " << decider_tag << " is not a lateral decision";
        lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
        // ADEBUG << decider_tag << " added obstacle " << Id();
        decisions_.push_back(decision);
        decider_tags_.push_back(decider_tag);
    }

    const std::string PathObstacle::DebugString() const
    {
        // std::stringstream ss;
        // ss << "PathObstacle id: " << id_;
        // for (std::size_t i = 0; i < decisions_.size(); ++i)
        // {
        //     ss << " decision: " << decisions_[i].DebugString() << ", made by "
        //        << decider_tags_[i];
        // }
        // if (lateral_decision_.object_tag_case() !=
        //     ObjectDecisionType::OBJECT_TAG_NOT_SET)
        // {
        //     ss << "lateral decision: " << lateral_decision_.ShortDebugString();
        // }
        // if (longitudinal_decision_.object_tag_case() !=
        //     ObjectDecisionType::OBJECT_TAG_NOT_SET)
        // {
        //     ss << "longitutional decision: "
        //        << longitudinal_decision_.ShortDebugString();
        // }
        // return ss.str();
    }

    const SLBoundary &PathObstacle::PerceptionSLBoundary() const
    {
        return perception_sl_boundary_;
    }

    void PathObstacle::SetStBoundary(const StBoundary &boundary)
    {
        st_boundary_ = boundary;
    }

    void PathObstacle::SetStBoundaryType(const StBoundary::BoundaryType type)
    {
        st_boundary_.SetBoundaryType(type);
    }

    void PathObstacle::EraseStBoundary() { st_boundary_ = StBoundary(); }

    void PathObstacle::SetReferenceLineStBoundary(const StBoundary &boundary)
    {
        reference_line_st_boundary_ = boundary;
    }

    void PathObstacle::SetReferenceLineStBoundaryType(
        const StBoundary::BoundaryType type)
    {
        reference_line_st_boundary_.SetBoundaryType(type);
    }

    void PathObstacle::EraseReferenceLineStBoundary()
    {
        reference_line_st_boundary_ = StBoundary();
    }

    bool PathObstacle::IsValidObstacle(
        const DetectedObject &perception_obstacle)
    {
        const double object_width = perception_obstacle.w;
        const double object_length = perception_obstacle.l;

        const double kMinObjectDimension = 1.0e-6;
        return !std::isnan(object_width) && !std::isnan(object_length) &&
               object_width > kMinObjectDimension &&
               object_length > kMinObjectDimension;
    }

    int PathObstacle::get_longitudinal_vaule(ObjectDecisionType tag)
    {
        if (tag == ignore)
            return 0;
        else if (tag == overtake)
            return 100;
        else if (tag == follow)
            return 300;
        else if (tag == yield)
            return 400;
        else if (tag == stop)
            return 500;
    }

    int PathObstacle::get_lateral_vaule(ObjectDecisionType tag)
    {
        if (tag == ignore)
            return 0;
        else if (tag == nudge)
            return 100;
        else if (tag == sidepass)
            return 200;
    }

} // namespace PlannerHNS
