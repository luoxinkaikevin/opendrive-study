#pragma once

#include <string>

#include "op_planner/vec2d.h"
#include <iostream>
#include <sstream>
namespace PlannerHNS
{

    class STPoint : public Vec2d
    {
    public:
        STPoint() = default;
        STPoint(const double s, const double t);
        explicit STPoint(const Vec2d &vec2d_point);
        std::string DebugString() const;

        double s() const;
        double t() const;
        void set_s(const double s);
        void set_t(const double t);
    };

} // namespace PlannerHNS
