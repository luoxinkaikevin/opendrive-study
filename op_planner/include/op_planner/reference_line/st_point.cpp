#include "op_planner/reference_line/st_point.h"

#include <iomanip>

namespace PlannerHNS
{

    STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

    STPoint::STPoint(const Vec2d &vec2d_point) : Vec2d(vec2d_point) {}

    double STPoint::s() const { return y; }

    double STPoint::t() const { return x; }

    void STPoint::set_s(const double s) { return set_y(s); }

    void STPoint::set_t(const double t) { return set_x(t); }

    std::string STPoint::DebugString() const
    {
        std::ostringstream temp_str;
        temp_str <<"t: "<< x << ",s: " << y;
        return temp_str.str();
    }

} // namespace PlannerHNS
