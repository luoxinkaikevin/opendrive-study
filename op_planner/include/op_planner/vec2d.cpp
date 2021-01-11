#include "op_planner/vec2d.h"

#include <cmath>

namespace PlannerHNS
{
    Vec2d Vec2d::CreateUnitVec2d(const double angle)
    {
        return Vec2d(cos(angle), sin(angle));
    }

    double Vec2d::Length() const { return std::hypot(x, y); }

    double Vec2d::LengthSquare() const { return x * x + y * y; }

    double Vec2d::Angle() const { return std::atan2(y, x); }

    void Vec2d::Normalize()
    {
        const double l = Length();
        if (l > vec::kMathEpsilon)
        {
            x /= l;
            y /= l;
        }
    }

    double Vec2d::DistanceTo(const Vec2d &other) const
    {
        return std::hypot(x - other.x, y - other.y);
    }

    double Vec2d::DistanceSquareTo(const Vec2d &other) const
    {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return dx * dx + dy * dy;
    }

    double Vec2d::CrossProd(const Vec2d &other) const
    {
        return x * other.y - y * other.x;
    }

    double Vec2d::InnerProd(const Vec2d &other) const
    {
        return x * other.x + y * other.y;
    }

    Vec2d Vec2d::rotate(const double angle) const
    {
        return Vec2d(x * cos(angle) - y * sin(angle),
                     x * sin(angle) + y * cos(angle));
    }

    Vec2d Vec2d::operator+(const Vec2d &other) const
    {
        return Vec2d(x + other.x, y + other.y);
    }

    Vec2d Vec2d::operator-(const Vec2d &other) const
    {
        return Vec2d(x - other.x, y - other.y);
    }

    Vec2d Vec2d::operator*(const double ratio) const
    {
        return Vec2d(x * ratio, y * ratio);
    }

    Vec2d Vec2d::operator/(const double ratio) const
    {
        CHECK_GT(std::abs(ratio), vec::kMathEpsilon);
        return Vec2d(x / ratio, y / ratio);
    }

    Vec2d &Vec2d::operator+=(const Vec2d &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2d &Vec2d::operator-=(const Vec2d &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2d &Vec2d::operator*=(const double ratio)
    {
        x *= ratio;
        y *= ratio;
        return *this;
    }

    Vec2d &Vec2d::operator/=(const double ratio)
    {
        CHECK_GT(std::abs(ratio), vec::kMathEpsilon);
        x /= ratio;
        y /= ratio;
        return *this;
    }

    bool Vec2d::operator==(const Vec2d &other) const
    {
        return (std::abs(x - other.x < vec::kMathEpsilon) &&
                std::abs(y - other.y < vec::kMathEpsilon));
    }

    Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

    std::string Vec2d::DebugString() const
    {
        // return util::StrCat("vec2d ( x = ", x, "  y = ", y, " )");
    }

} // namespace PlannerHNS