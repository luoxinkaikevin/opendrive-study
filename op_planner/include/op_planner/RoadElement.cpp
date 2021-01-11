#include "op_planner/RoadElement.h"

namespace PlannerHNS
{

    const double kMathEpsilon = 1e-7;

    GPSPoint::GPSPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        a = 0;
    }

    GPSPoint::GPSPoint(const double &x, const double &y, const double &z, const double &a)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;
    }

    GPSPoint GPSPoint::CreateUnitVec2d(const double angle)
    {
        return GPSPoint(cos(angle), sin(angle), 0, 0);
    }

    double GPSPoint::LengthToOrigin() const { return std::hypot(this->x, this->y); }

    double GPSPoint::LengthSquare() const { return this->x * this->x + this->y * this->y; }

    double GPSPoint::AngleWithOrigin() const { return std::atan2(this->y, this->x); }

    double GPSPoint::DistanceTo(const GPSPoint &other) const
    {
        return std::hypot(this->x - other.x, this->y - other.y);
    }

    void GPSPoint::Normalize()
    {
        const double l = Length();
        if (l > kMathEpsilon)
        {
            x /= l;
            y /= l;
        }
    }

    double GPSPoint::DistanceSquareTo(const GPSPoint &other) const
    {
        const double dx = this->x - other.x;
        const double dy = this->y - other.y;
        return dx * dx + dy * dy;
    }

    double GPSPoint::CrossProd(const GPSPoint &other) const
    {
        return x * other.y - y * other.x;
    }

    double GPSPoint::InnerProd(const GPSPoint &other) const
    {
        return x * other.x + y * other.y;
    }

    GPSPoint GPSPoint::rotate(const double angle) const
    {
        return GPSPoint(x * cos(angle) - y * sin(angle),
                        x * sin(angle) + y * cos(angle), z, a);
    }

    GPSPoint GPSPoint::operator+(const GPSPoint &other) const
    {
        return GPSPoint(x + other.x, y + other.y, z, a);
    }

    GPSPoint GPSPoint::operator-(const GPSPoint &other) const
    {
        return GPSPoint(x - other.x, y - other.y, z, a);
    }

    GPSPoint GPSPoint::operator*(const double ratio) const
    {
        return GPSPoint(x * ratio, y * ratio, z, a);
    }

    GPSPoint GPSPoint::operator/(const double ratio) const
    {
        return GPSPoint(x / ratio, y / ratio, z, a);
    }

    GPSPoint &GPSPoint::operator+=(const GPSPoint &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    GPSPoint &GPSPoint::operator-=(const GPSPoint &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    GPSPoint &GPSPoint::operator*=(const double ratio)
    {
        x *= ratio;
        y *= ratio;
        return *this;
    }

    GPSPoint &GPSPoint::operator/=(const double ratio)
    {
        x /= ratio;
        y /= ratio;
        return *this;
    }

    bool GPSPoint::operator==(const GPSPoint &other) const
    {
        return (std::abs(x - other.x) < kMathEpsilon &&
                std::abs(y - other.y) < kMathEpsilon);
    }

    std::string GPSPoint::ToString() const
    {
        std::stringstream str;
        str.precision(12);
        str << "X:" << this->x << ", Y:" << this->y << ", Z:" << this->z << ", A:" << this->a << std::endl;
        return str.str();
    }
    // ------------------------------------------------------------------------------------

} // namespace PlannerHNS