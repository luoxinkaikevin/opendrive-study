#pragma once

#include <cmath>
#include <string>
#include "op_planner/log.h"

namespace PlannerHNS
{
    namespace vec
    {
        constexpr double kMathEpsilon = 1e-10;
    }
    //

    /**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
    class Vec2d
    {
    public:
        //! Constructor which takes x- and y-coordinates.
        constexpr Vec2d(const double x, const double y) noexcept : x(x), y(y) {}

        //! Constructor returning the zero vector.
        constexpr Vec2d() noexcept : Vec2d(0, 0) {}

        //! Creates a unit-vector with a given angle to the positive x semi-axis
        static Vec2d CreateUnitVec2d(const double angle);

        //! Gets the length of the vector
        double Length() const;

        //! Gets the squared length of the vector
        double LengthSquare() const;

        //! Gets the angle between the vector and the positive x semi-axis
        double Angle() const;

        //! Returns the unit vector that is co-linear with this vector
        void Normalize();

        //! Returns the distance to the given vector
        double DistanceTo(const Vec2d &other) const;

        //! Returns the squared distance to the given vector
        double DistanceSquareTo(const Vec2d &other) const;

        //! Returns the "cross" product between these two Vec2d (non-standard).
        double CrossProd(const Vec2d &other) const;

        //! Returns the inner product between these two Vec2d.
        double InnerProd(const Vec2d &other) const;

        //! rotate the vector by angle.
        Vec2d rotate(const double angle) const;

        //! Sums two Vec2d
        Vec2d operator+(const Vec2d &other) const;

        //! Subtracts two Vec2d
        Vec2d operator-(const Vec2d &other) const;

        //! Multiplies Vec2d by a scalar
        Vec2d operator*(const double ratio) const;

        //! Divides Vec2d by a scalar
        Vec2d operator/(const double ratio) const;

        //! Sums another Vec2d to the current one
        Vec2d &operator+=(const Vec2d &other);

        //! Subtracts another Vec2d to the current one
        Vec2d &operator-=(const Vec2d &other);

        //! Multiplies this Vec2d by a scalar
        Vec2d &operator*=(const double ratio);

        //! Divides this Vec2d by a scalar
        Vec2d &operator/=(const double ratio);

        //! Compares two Vec2d
        bool operator==(const Vec2d &other) const;

        //! Returns a human-readable string representing this object
        std::string DebugString() const;

        void set_y(double y_) { y = y_; };
        void set_x(double x_) { x = x_; };
        double x;
        double y;
    };

    //! Multiplies the given Vec2d by a given scalar
    Vec2d operator*(const double ratio, const Vec2d &vec);

} // namespace PlannerHNS
