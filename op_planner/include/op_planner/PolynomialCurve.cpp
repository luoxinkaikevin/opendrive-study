#include "op_planner/PolynomialCurve.h"

// #include <ros/ros.h>

namespace PlannerHNS
{
    QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
        const std::array<double, 3> &start, const std::array<double, 3> &end,
        const double param)
        : QuinticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                                   end[2], param) {}

    QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
        const double x0, const double dx0, const double ddx0, const double x1,
        const double dx1, const double ddx1, const double param)
    {
        ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
        start_condition_[0] = x0;
        start_condition_[1] = dx0;
        start_condition_[2] = ddx0;
        end_condition_[0] = x1;
        end_condition_[1] = dx1;
        end_condition_[2] = ddx1;
        param_ = param;
    }

    QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
        const QuinticPolynomialCurve1d &other)
    {
        param_ = other.param_;
        coef_ = other.coef_;
        return;
    }

    double QuinticPolynomialCurve1d::Evaluate(const uint32_t order,
                                              const double p) const
    {
        switch (order)
        {
        case 0:
        {
            return ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
                    coef_[1]) *
                       p +
                   coef_[0];
        }
        case 1:
        {
            return (((5.0 * coef_[5] * p + 4.0 * coef_[4]) * p + 3.0 * coef_[3]) * p +
                    2.0 * coef_[2]) *
                       p +
                   coef_[1];
        }
        case 2:
        {
            return (((20.0 * coef_[5] * p + 12.0 * coef_[4]) * p) + 6.0 * coef_[3]) *
                       p +
                   2.0 * coef_[2];
        }
        case 3:
        {
            return (60.0 * coef_[5] * p + 24.0 * coef_[4]) * p + 6.0 * coef_[3];
        }
        case 4:
        {
            return 120.0 * coef_[5] * p + 24.0 * coef_[4];
        }
        case 5:
        {
            return 120.0 * coef_[5];
        }
        default:
            return 0.0;
        }
    }

    void QuinticPolynomialCurve1d::ComputeCoefficients(
        const double x0, const double dx0, const double ddx0, const double x1,
        const double dx1, const double ddx1, const double p)
    {
        // CHECK_GT(p, 0.0);

        coef_[0] = x0;
        coef_[1] = dx0;
        coef_[2] = ddx0 / 2.0;

        const double p2 = p * p;
        const double p3 = p * p2;

        // the direct analytical method is at least 6 times faster than using matrix
        // inversion.
        const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
        const double c1 = (dx1 - ddx0 * p - dx0) / p2;
        const double c2 = (ddx1 - ddx0) / p;

        coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
        coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
        coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
    }

    std::string QuinticPolynomialCurve1d::ToString() const
    {
        std::ostringstream temp_str;
        temp_str << start_condition_[0] << "," << start_condition_[1] << "," << start_condition_[2] << ","
                 << end_condition_[0] << "," << end_condition_[1] << "," << end_condition_[2] << ","
                 << coef_[0] << "," << coef_[1] << "," << coef_[2] << ","
                 << coef_[3] << "," << coef_[4] << "," << coef_[5] << ","
                 << param_;

        // temp_str
        //     << "end d," << end_condition_[0] << ",s," << param_;

        return temp_str.str();
    }

    QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
        const std::array<double, 3> &start, const std::array<double, 2> &end,
        const double param)
        : QuarticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                                   param) {}

    QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
        const double x0, const double dx0, const double ddx0, const double dx1,
        const double ddx1, const double param)
    {
        param_ = param;
        start_condition_[0] = x0;
        start_condition_[1] = dx0;
        start_condition_[2] = ddx0;
        end_condition_[0] = dx1;
        end_condition_[1] = ddx1;
        ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
    }

    QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
        const QuarticPolynomialCurve1d &other)
    {
        param_ = other.param_;
        coef_ = other.coef_;
    }

    double QuarticPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                              const double p) const
    {
        switch (order)
        {
        case 0:
        {
            return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
                   coef_[0];
        }
        case 1:
        {
            return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
                   coef_[1];
        }
        case 2:
        {
            return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
        }
        case 3:
        {
            return 24.0 * coef_[4] * p + 6.0 * coef_[3];
        }
        case 4:
        {
            return 24.0 * coef_[4];
        }
        default:
            return 0.0;
        }
    }

    void QuarticPolynomialCurve1d::ComputeCoefficients(
        const double x0, const double dx0, const double ddx0, const double dx1,
        const double ddx1, const double p)
    {

        coef_[0] = x0;
        coef_[1] = dx0;
        coef_[2] = 0.5 * ddx0;

        double b0 = dx1 - ddx0 * p - dx0;
        double b1 = ddx1 - ddx0;

        double p2 = p * p;
        double p3 = p2 * p;

        coef_[3] = (3 * b0 - b1 * p) / (3 * p2);
        coef_[4] = (-2 * b0 + b1 * p) / (4 * p3);
    }

    std::string QuarticPolynomialCurve1d::ToString() const
    {
        std::ostringstream temp_str;
        temp_str << start_condition_[0] << "," << start_condition_[1] << "," << start_condition_[2] << ","
                 << end_condition_[0] << "," << end_condition_[1] << ","
                 << coef_[0] << "," << coef_[1] << "," << coef_[2] << ","
                 << coef_[3] << "," << coef_[4] << "," << param_;

        // temp_str
        //     << ",end v," << end_condition_[0] << ",t," << param_;

        return temp_str.str();
    }

    CubicPolynomialCurve1d::CubicPolynomialCurve1d(
        const std::array<double, 3> &start, const double end, const double param)
        : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

    CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                                   const double dx0,
                                                   const double ddx0,
                                                   const double x1,
                                                   const double param)
    {
        ComputeCoefficients(x0, dx0, ddx0, x1, param);
        param_ = param;
        start_condition_[0] = x0;
        start_condition_[1] = dx0;
        start_condition_[2] = ddx0;
        end_condition_ = x1;
    }

    double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                            const double p) const
    {
        switch (order)
        {
        case 0:
        {
            return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
        }
        case 1:
        {
            return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
        }
        case 2:
        {
            return 6.0 * coef_[3] * p + 2.0 * coef_[2];
        }
        case 3:
        {
            return 6.0 * coef_[3];
        }
        default:
            return 0.0;
        }
    }

    std::string CubicPolynomialCurve1d::ToString() const
    {
        std::ostringstream temp_str;
        temp_str << start_condition_[0] << "," << start_condition_[1] << "," << start_condition_[2] << ","
                 << end_condition_ << ","
                 << coef_[0] << "," << coef_[1] << "," << coef_[2] << ","
                 << coef_[3] << "," << param_;
        return temp_str.str();
    }

    void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                     const double dx0,
                                                     const double ddx0,
                                                     const double x1,
                                                     const double param)
    {
        // DCHECK(param > 0.0);
        const double p2 = param * param;
        const double p3 = param * p2;
        coef_[0] = x0;
        coef_[1] = dx0;
        coef_[2] = 0.5 * ddx0;
        coef_[3] = (x1 - x0 - dx0 * param - coef_[2] * p2) / p3;
    }

    LatticeTrajectory1d::LatticeTrajectory1d(
        std::shared_ptr<Curve1d> ptr_trajectory1d)
    {
        ptr_trajectory1d_ = ptr_trajectory1d;
    }

    double LatticeTrajectory1d::Evaluate(const std::uint32_t order,
                                         const double param) const
    {
        double param_length = ptr_trajectory1d_->ParamLength();
        if (param < param_length)
        {
            return ptr_trajectory1d_->Evaluate(order, param);
        }

        // do constant acceleration extrapolation;
        // to align all the trajectories with time.
        double p = ptr_trajectory1d_->Evaluate(0, param_length);
        double v = ptr_trajectory1d_->Evaluate(1, param_length);
        double a = ptr_trajectory1d_->Evaluate(2, param_length);

        double t = param - param_length;

        switch (order)
        {
        case 0:
            return p + v * t + 0.5 * a * t * t;
        case 1:
            return v + a * t;
        case 2:
            return a;
        default:
            return 0.0;
        }
    }

    double LatticeTrajectory1d::ParamLength() const
    {
        return ptr_trajectory1d_->ParamLength();
    }

    std::string LatticeTrajectory1d::ToString() const
    {
        return ptr_trajectory1d_->ToString();
    }

    bool LatticeTrajectory1d::has_target_position() const
    {
        return has_target_position_;
    }

    bool LatticeTrajectory1d::has_target_velocity() const
    {
        return has_target_velocity_;
    }

    bool LatticeTrajectory1d::has_target_time() const { return has_target_time_; }

    double LatticeTrajectory1d::target_position() const
    {
        // CHECK(has_target_position_);
        return target_position_;
    }

    double LatticeTrajectory1d::target_velocity() const
    {
        // CHECK(has_target_velocity_);
        return target_velocity_;
    }

    double LatticeTrajectory1d::target_time() const
    {
        // CHECK(has_target_time_);
        return target_time_;
    }

    void LatticeTrajectory1d::set_target_position(double target_position)
    {
        target_position_ = target_position;
        has_target_position_ = true;
    }

    void LatticeTrajectory1d::set_target_velocity(double target_velocity)
    {
        target_velocity_ = target_velocity;
        has_target_velocity_ = true;
    }

    void LatticeTrajectory1d::set_target_time(double target_time)
    {
        target_time_ = target_time;
        has_target_time_ = true;
    }

    DiscretizedTrajectory::DiscretizedTrajectory(
        const std::vector<WayPoint> &trajectory_points)
    {
        trajectory_points_ = trajectory_points;
    }

    WayPoint DiscretizedTrajectory::Evaluate(const double relative_time) const
    {
        auto comp = [](const WayPoint &p, const double relative_time) {
            return p.timeCost < relative_time;
        };

        auto it_lower =
            std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                             relative_time, comp);

        if (it_lower == trajectory_points_.begin())
        {
            return trajectory_points_.front();
        }
        else if (it_lower == trajectory_points_.end())
        {
            return trajectory_points_.back();
        }
        return PlanningHelpers::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
    }

    std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
        const double relative_time) const
    {
        if (relative_time >= trajectory_points_.back().timeCost)
        {
            return trajectory_points_.size() - 1;
        }
        auto func = [](const WayPoint &tp, const double relative_time) {
            return tp.timeCost < relative_time;
        };
        auto it_lower =
            std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                             relative_time, func);
        return std::distance(trajectory_points_.begin(), it_lower);
    }

    std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
        const GPSPoint &position) const
    {
        double dist_sqr_min = std::numeric_limits<double>::max();
        std::uint32_t index_min = 0;
        for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i)
        {
            const GPSPoint curr_point(
                trajectory_points_[i].pos.x, trajectory_points_[i].pos.y, 0, 0);

            const double dist_sqr = curr_point.DistanceSquareTo(position);
            if (dist_sqr < dist_sqr_min)
            {
                dist_sqr_min = dist_sqr;
                index_min = i;
            }
        }
        return index_min;
    }

    void DiscretizedTrajectory::AppendWayPoint(const WayPoint &trajectory_point)
    {
        trajectory_points_.push_back(trajectory_point);
    }

    const WayPoint &DiscretizedTrajectory::WayPointAt(const std::uint32_t index) const
    {
        return trajectory_points_[index];
    }

    WayPoint DiscretizedTrajectory::StartPoint() const
    {

        return trajectory_points_.front();
    }

    double DiscretizedTrajectory::GetTemporalLength() const
    {
        return trajectory_points_.back().timeCost -
               trajectory_points_.front().timeCost;
    }

    double DiscretizedTrajectory::GetSpatialLength() const
    {
        return trajectory_points_.back().s() -
               trajectory_points_.front().s();
    }

    ConstantAccelerationTrajectory1d::ConstantAccelerationTrajectory1d(
        const double start_s, const double start_v)
    {
        s_.push_back(start_s);
        v_.push_back(start_v);
        a_.push_back(0.0);
        t_.push_back(0.0);
    }

    void ConstantAccelerationTrajectory1d::AppendSegment(
        const double a, const double t_duration)
    {
        double s0 = s_.back();
        double v0 = v_.back();
        double t0 = t_.back();

        double v1 = v0 + a * t_duration;
        // CHECK(v1 >= -FLAGS_lattice_epsilon);

        double delta_s = (v0 + v1) * t_duration * 0.5;
        double s1 = s0 + delta_s;
        double t1 = t0 + t_duration;

        // CHECK(s1 >= s0 - FLAGS_lattice_epsilon);
        s1 = std::max(s1, s0);

        s_.push_back(s1);
        v_.push_back(v1);
        a_.push_back(a);
        t_.push_back(t1);
    }

    void ConstantAccelerationTrajectory1d::PopSegment()
    {
        if (a_.size() > 0)
        {
            s_.pop_back();
            v_.pop_back();
            a_.pop_back();
            t_.pop_back();
        }
    }

    double ConstantAccelerationTrajectory1d::ParamLength() const
    {
        // CHECK_GT(t_.size(), 1);
        return t_.back() - t_.front();
    }

    std::string ConstantAccelerationTrajectory1d::ToString() const
    {
        // return apollo::common::util::StrCat(apollo::common::util::PrintIter(s_, "\t"),
        //                                     apollo::common::util::PrintIter(t_, "\t"),
        //                                     apollo::common::util::PrintIter(v_, "\t"),
        //                                     apollo::common::util::PrintIter(a_, "\t"),
        //                                     "\n");
    }

    double ConstantAccelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                      const double param) const
    {
        // CHECK_GT(t_.size(), 1);
        // CHECK(t_.front() <= param && param <= t_.back());

        switch (order)
        {
        case 0:
            return Evaluate_s(param);
        case 1:
            return Evaluate_v(param);
        case 2:
            return Evaluate_a(param);
        case 3:
            return Evaluate_j(param);
        }
        return 0.0;
    }

    double ConstantAccelerationTrajectory1d::Evaluate_s(const double t) const
    {
        auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
        auto index = std::distance(t_.begin(), it_lower);

        double s0 = s_[index - 1];
        double v0 = v_[index - 1];
        double t0 = t_[index - 1];

        double v1 = v_.back();
        double t1 = t_.back();

        double v = lerp(v0, t0, v1, t1, t);
        double s = (v0 + v) * (t - t0) * 0.5 + s0;
        return s;
    }

    double ConstantAccelerationTrajectory1d::Evaluate_v(const double t) const
    {
        auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
        auto index = std::distance(t_.begin(), it_lower);

        double v0 = v_[index - 1];
        double t0 = t_[index - 1];

        double v1 = v_.back();
        double t1 = t_.back();

        double v = lerp(v0, t0, v1, t1, t);

        return v;
    }

    double ConstantAccelerationTrajectory1d::Evaluate_a(const double t) const
    {
        auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
        auto index = std::distance(t_.begin(), it_lower);
        return a_[index - 1];
    }

    double ConstantAccelerationTrajectory1d::Evaluate_j(const double t) const
    {
        return 0.0;
    }

    std::array<double, 4> ConstantAccelerationTrajectory1d::Evaluate(
        const double t) const
    {

        auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
        auto index = std::distance(t_.begin(), it_lower);

        double s0 = s_[index - 1];
        double v0 = v_[index - 1];
        double t0 = t_[index - 1];

        double v1 = v_.back();
        double t1 = t_.back();

        double v = lerp(v0, t0, v1, t1, t);
        double s = (v0 + v) * (t - t0) * 0.5 + s0;

        double a = a_[index - 1];
        double j = 0.0;

        return {{s, v, a, j}};
    }

    std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(
        const double s_target, const double s_curr, const double v_target,
        const double v_curr, const double a_comfort, const double d_comfort)
    {

        std::shared_ptr<ConstantAccelerationTrajectory1d> ptr_trajectory =
            std::make_shared<ConstantAccelerationTrajectory1d>(s_curr, v_curr);

        double s_dist = s_target - s_curr;

        double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);

        // if cannot stop using comfort deceleration, then brake in the beginning.
        if (comfort_stop_dist > s_dist)
        {
            double stop_d = ComputeStopDeceleration(s_dist, v_curr);
            double stop_t = v_curr / stop_d;
            ptr_trajectory->AppendSegment(-stop_d, stop_t);
            return ptr_trajectory;
        }

        // otherwise, the vehicle can stop from current speed with comfort brake.
        if (v_curr > v_target)
        {
            double t_cruise = (s_dist - comfort_stop_dist) / v_target;
            double t_rampdown = (v_curr - v_target) / d_comfort;
            double t_dec = v_target / d_comfort;

            ptr_trajectory->AppendSegment(-d_comfort, t_rampdown);
            ptr_trajectory->AppendSegment(0.0, t_cruise);
            ptr_trajectory->AppendSegment(-d_comfort, t_dec);
            return ptr_trajectory;
        }
        else
        {
            double t_rampup = (v_target - v_curr) / a_comfort;
            double t_rampdown = (v_target - v_curr) / d_comfort;
            double s_ramp = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5;

            double s_rest = s_dist - s_ramp - comfort_stop_dist;
            if (s_rest > 0)
            {
                double t_cruise = s_rest / v_target;
                double t_dec = v_target / d_comfort;

                // construct the trajectory
                ptr_trajectory->AppendSegment(a_comfort, t_rampup);
                ptr_trajectory->AppendSegment(0.0, t_cruise);
                ptr_trajectory->AppendSegment(-d_comfort, t_dec);
                return ptr_trajectory;
            }
            else
            {
                double s_rampup_rampdown = s_dist - comfort_stop_dist;
                double v_max = std::sqrt(
                    v_curr * v_curr + 2.0 * a_comfort * d_comfort * s_rampup_rampdown / (a_comfort + d_comfort));

                double t_acc = (v_max - v_curr) / a_comfort;
                double t_dec = v_max / d_comfort;

                // construct the trajectory
                ptr_trajectory->AppendSegment(a_comfort, t_acc);
                ptr_trajectory->AppendSegment(-d_comfort, t_dec);
                return ptr_trajectory;
            }
        }
    }

    double PiecewiseBrakingTrajectoryGenerator::ComputeStopDistance(const double v,
                                                                    const double dec)
    {
        CHECK(dec > 0.0);
        return v * v / dec * 0.5;
    }

    double PiecewiseBrakingTrajectoryGenerator::ComputeStopDeceleration(
        const double dist, const double v)
    {
        return v * v / dist * 0.5;
    }

    PublishableTrajectory::PublishableTrajectory(
        const double header_time,
        const DiscretizedTrajectory &discretized_trajectory)
        : DiscretizedTrajectory(discretized_trajectory),
          header_time_(header_time) {}

    double PublishableTrajectory::header_time() const { return header_time_; }

} // namespace PlannerHNS