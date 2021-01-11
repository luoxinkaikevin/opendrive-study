
#ifndef QUNITICPOLYNOMIALCURVE_H_
#define QUNITICPOLYNOMIALCURVE_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include "op_planner/log.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MatrixOperations.h"

namespace PlannerHNS
{
    class Curve1d
    {
    public:
        Curve1d() = default;

        virtual ~Curve1d() = default;

        virtual double Evaluate(const std::uint32_t order,
                                const double param) const = 0;

        virtual double ParamLength() const = 0;

        virtual std::string ToString() const = 0;
    };
    class PolynomialCurve1d : public Curve1d
    {
    public:
        PolynomialCurve1d() = default;
        virtual ~PolynomialCurve1d() = default;

    protected:
        double param_ = 0.0;
    };

    // 1D quintic polynomial curve:
    // (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
    class QuinticPolynomialCurve1d : public PolynomialCurve1d
    {
    public:
        QuinticPolynomialCurve1d() = default;

        QuinticPolynomialCurve1d(const std::array<double, 3> &start,
                                 const std::array<double, 3> &end,
                                 const double param);

        QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                                 const double x1, const double dx1, const double ddx1,
                                 const double param);

        QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d &other);

        virtual ~QuinticPolynomialCurve1d() = default;

        double Evaluate(const std::uint32_t order, const double p) const override;

        double ParamLength() const { return param_; }
        std::string ToString() const override;

    protected:
        void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                                 const double x1, const double dx1, const double ddx1,
                                 const double param);

        // f = sum(coef_[i] * x^i), i from 0 to 5
        std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
        std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
    };

    // 1D quartic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (dx1, ddx1)
    class QuarticPolynomialCurve1d : public PolynomialCurve1d
    {
    public:
        QuarticPolynomialCurve1d() = default;

        QuarticPolynomialCurve1d(const std::array<double, 3> &start,
                                 const std::array<double, 2> &end,
                                 const double param);

        QuarticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                                 const double dx1, const double ddx1,
                                 const double param);

        QuarticPolynomialCurve1d(const QuarticPolynomialCurve1d &other);

        virtual ~QuarticPolynomialCurve1d() = default;

        double Evaluate(const std::uint32_t order, const double p) const override;

        double ParamLength() const override { return param_; }
        std::string ToString() const override;

    private:
        void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                                 const double dx1, const double ddx1,
                                 const double param);

        std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
        std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
        std::array<double, 2> end_condition_ = {{0.0, 0.0}};
    };

    class LatticeTrajectory1d : public Curve1d
    {
    public:
        explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d);

        virtual ~LatticeTrajectory1d() = default;

        virtual double Evaluate(const std::uint32_t order, const double param) const;

        virtual double ParamLength() const;

        virtual std::string ToString() const;

        bool has_target_position() const;

        bool has_target_velocity() const;

        bool has_target_time() const;

        double target_position() const;

        double target_velocity() const;

        double target_time() const;

        void set_target_position(double target_position);

        void set_target_velocity(double target_velocity);

        void set_target_time(double target_time);

    private:
        std::shared_ptr<Curve1d> ptr_trajectory1d_;

        double target_position_ = 0.0;

        double target_velocity_ = 0.0;

        double target_time_ = 0.0;

        bool has_target_position_ = false;

        bool has_target_velocity_ = false;

        bool has_target_time_ = false;
    };

    class CubicPolynomialCurve1d : public PolynomialCurve1d
    {
    public:
        CubicPolynomialCurve1d() = default;
        virtual ~CubicPolynomialCurve1d() = default;

        CubicPolynomialCurve1d(const std::array<double, 3> &start, const double end,
                               const double param);

        /**
   * x0 is the value when f(x = 0);
   * dx0 is the value when f'(x = 0);
   * ddx0 is the value when f''(x = 0);
   * f(x = param) = x1
   */
        CubicPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                               const double x1, const double param);

        double Evaluate(const std::uint32_t order, const double p) const override;

        double ParamLength() const { return param_; }
        std::string ToString() const override;

    private:
        void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                                 const double x1, const double param);
        std::array<double, 4> coef_ = {{0.0, 0.0, 0.0, 0.0}};
        std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
        double end_condition_ = 0.0;
    };

    class Trajectory
    {
    public:
        Trajectory() = default;

        virtual ~Trajectory() = default;

        virtual WayPoint Evaluate(
            const double relative_time) const = 0;

        virtual WayPoint StartPoint() const = 0;

        virtual double GetTemporalLength() const = 0;

        virtual double GetSpatialLength() const = 0;
    };

    class DiscretizedTrajectory : public Trajectory
    {
    public:
        DiscretizedTrajectory() = default;

        explicit DiscretizedTrajectory(
            const std::vector<WayPoint> &trajectory_points);

        void SetTrajectoryPoints(
            const std::vector<WayPoint> &trajectory_points);

        virtual ~DiscretizedTrajectory() = default;

        WayPoint StartPoint() const override;

        double GetTemporalLength() const override;

        double GetSpatialLength() const override;

        WayPoint Evaluate(const double relative_time) const override;

        virtual uint32_t QueryNearestPoint(const double relative_time) const;

        virtual uint32_t QueryNearestPoint(const GPSPoint &position) const;

        virtual void AppendWayPoint(const WayPoint &trajectory_point);

        template <typename Iter>
        void PrependTrajectoryPoints(Iter begin, Iter end)
        {
            trajectory_points_.insert(trajectory_points_.begin(), begin, end);
        }

        const WayPoint &WayPointAt(const std::uint32_t index) const;

        uint32_t NumOfPoints() const;

        const std::vector<WayPoint> &trajectory_points() const;

        std::vector<WayPoint> &trajectory_points();

        virtual void Clear();

    protected:
        std::vector<WayPoint> trajectory_points_;
    };

    inline std::uint32_t DiscretizedTrajectory::NumOfPoints() const
    {
        return trajectory_points_.size();
    }

    inline const std::vector<WayPoint> &
    DiscretizedTrajectory::trajectory_points() const
    {
        return trajectory_points_;
    }

    inline std::vector<WayPoint> &
    DiscretizedTrajectory::trajectory_points()
    {
        return trajectory_points_;
    }

    inline void DiscretizedTrajectory::SetTrajectoryPoints(
        const std::vector<WayPoint> &trajectory_points)
    {
        trajectory_points_ = trajectory_points;
    }

    inline void DiscretizedTrajectory::Clear() { trajectory_points_.clear(); }


    class ConstantAccelerationTrajectory1d : public Curve1d
    {
    public:
        ConstantAccelerationTrajectory1d(const double start_s, const double start_v);

        virtual ~ConstantAccelerationTrajectory1d() = default;

        void AppendSegment(const double a, const double t_duration);

        void PopSegment();

        double ParamLength() const override;

        std::string ToString() const override;

        double Evaluate(const std::uint32_t order, const double param) const override;

        std::array<double, 4> Evaluate(const double t) const;

    private:
        double Evaluate_s(const double t) const;

        double Evaluate_v(const double t) const;

        double Evaluate_a(const double t) const;

        double Evaluate_j(const double t) const;

    private:
        // accumulated s
        std::vector<double> s_;

        std::vector<double> v_;

        // accumulated t
        std::vector<double> t_;

        std::vector<double> a_;
    };

    class PiecewiseBrakingTrajectoryGenerator
    {
    public:
        PiecewiseBrakingTrajectoryGenerator() = delete;

        static std::shared_ptr<Curve1d> Generate(
            const double s_target, const double s_curr,
            const double v_target, const double v_curr,
            const double a_comfort, const double d_comfort);

        static double ComputeStopDistance(const double v,
                                          const double dec);

        static double ComputeStopDeceleration(const double dist,
                                              const double v);
    };

    class PublishableTrajectory : public DiscretizedTrajectory
    {
    public:
        PublishableTrajectory() = default;

        PublishableTrajectory(const double header_time,
                              const DiscretizedTrajectory &discretized_trajectory);
        double header_time() const;
    private:
        double header_time_ = 0.0;
    };

} /* namespace PlannerHNS */

#endif
