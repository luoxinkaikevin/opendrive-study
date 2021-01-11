#include "box2d.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace PlannerHNS
{
    namespace math
    {

        namespace
        {
            constexpr double kMathEpsilon = 1e-10;
            double PtSegDistance(double query_x, double query_y, double start_x,
                                 double start_y, double end_x, double end_y,
                                 double length)
            {
                const double x0 = query_x - start_x;
                const double y0 = query_y - start_y;
                const double dx = end_x - start_x;
                const double dy = end_y - start_y;
                const double proj = x0 * dx + y0 * dy;
                if (proj <= 0.0)
                {
                    return hypot(x0, y0);
                }
                if (proj >= length * length)
                {
                    return hypot(x0 - dx, y0 - dy);
                }
                return std::abs(x0 * dy - y0 * dx) / length;
            }
            bool IsWithin(double val, double bound1, double bound2)
            {
                if (bound1 > bound2)
                {
                    std::swap(bound1, bound2);
                }
                return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
            }

            double CrossProd(const GPSPoint &start_point, const GPSPoint &end_point_1,
                             const GPSPoint &end_point_2)
            {
                return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
            }

        } // namespace
        Polygon2d::Polygon2d(const Box2d &box)
        {
            box.GetAllCorners(&points_);
            BuildFromPoints();
        }

        Polygon2d::Polygon2d(std::vector<GPSPoint> points) : points_(std::move(points))
        {
            BuildFromPoints();
        }

        double Polygon2d::DistanceTo(const GPSPoint &point) const
        {
            CHECK_GE(points_.size(), 3);
            if (IsPointIn(point))
            {
                return 0.0;
            }
            double distance = std::numeric_limits<double>::infinity();
            for (int i = 0; i < num_points_; ++i)
            {
                distance = std::min(distance, line_segments_[i].DistanceTo(point));
            }
            return distance;
        }

        double Polygon2d::DistanceSquareTo(const GPSPoint &point) const
        {
            CHECK_GE(points_.size(), 3);
            if (IsPointIn(point))
            {
                return 0.0;
            }
            double distance_sqr = std::numeric_limits<double>::infinity();
            for (int i = 0; i < num_points_; ++i)
            {
                distance_sqr =
                    std::min(distance_sqr, line_segments_[i].DistanceSquareTo(point));
            }
            return distance_sqr;
        }

        double Polygon2d::DistanceTo(const Box2d &box) const
        {
            CHECK_GE(points_.size(), 3);
            return DistanceTo(Polygon2d(box));
        }

        double Polygon2d::DistanceTo(const LineSegment2d &line_segment) const
        {
            if (line_segment.length() <= kMathEpsilon)
            {
                return DistanceTo(line_segment.start());
            }
            CHECK_GE(points_.size(), 3);
            if (IsPointIn(line_segment.center()))
            {
                return 0.0;
            }
            if (std::any_of(line_segments_.begin(), line_segments_.end(),
                            [&](const LineSegment2d &poly_seg) {
                                return poly_seg.HasIntersect(line_segment);
                            }))
            {
                return 0.0;
            }

            double distance = std::min(DistanceTo(line_segment.start()),
                                       DistanceTo(line_segment.end()));
            for (int i = 0; i < num_points_; ++i)
            {
                distance = std::min(distance, line_segment.DistanceTo(points_[i]));
            }
            return distance;
        }

        double Polygon2d::DistanceTo(const Polygon2d &polygon) const
        {
            CHECK_GE(points_.size(), 3);
            CHECK_GE(polygon.num_points(), 3);

            if (IsPointIn(polygon.points()[0]))
            {
                return 0.0;
            }
            if (polygon.IsPointIn(points_[0]))
            {
                return 0.0;
            }
            double distance = std::numeric_limits<double>::infinity();
            for (int i = 0; i < num_points_; ++i)
            {
                distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
            }
            return distance;
        }

        double Polygon2d::DistanceToBoundary(const GPSPoint &point) const
        {
            double distance = std::numeric_limits<double>::infinity();
            for (int i = 0; i < num_points_; ++i)
            {
                distance = std::min(distance, line_segments_[i].DistanceTo(point));
            }
            return distance;
        }

        bool Polygon2d::IsPointOnBoundary(const GPSPoint &point) const
        {
            CHECK_GE(points_.size(), 3);
            return std::any_of(
                line_segments_.begin(), line_segments_.end(),
                [&](const LineSegment2d &poly_seg) { return poly_seg.IsPointIn(point); });
        }

        bool Polygon2d::IsPointIn(const GPSPoint &point) const
        {
            CHECK_GE(points_.size(), 3);
            if (IsPointOnBoundary(point))
            {
                return true;
            }
            int j = num_points_ - 1;
            int c = 0;
            for (int i = 0; i < num_points_; ++i)
            {
                if ((points_[i].y > point.y) != (points_[j].y > point.y))
                {
                    const double side = CrossProd(point, points_[i], points_[j]);
                    if (points_[i].y < points_[j].y ? side > 0.0 : side < 0.0)
                    {
                        ++c;
                    }
                }
                j = i;
            }
            return c & 1;
        }

        bool Polygon2d::HasOverlap(const Polygon2d &polygon) const
        {
            CHECK_GE(points_.size(), 3);
            if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
                polygon.max_y() < min_y() || polygon.min_y() > max_y())
            {
                return false;
            }
            return DistanceTo(polygon) <= kMathEpsilon;
        }

        void Polygon2d::ExtremePoints(const double heading, GPSPoint *const first,
                                      GPSPoint *const last) const
        {
            CHECK_GE(points_.size(), 3);
            CHECK_NOTNULL(first);
            CHECK_NOTNULL(last);

            const GPSPoint direction_vec = GPSPoint::CreateUnitVec2d(heading);
            double min_proj = std::numeric_limits<double>::infinity();
            double max_proj = -std::numeric_limits<double>::infinity();
            for (const auto &pt : points_)
            {
                const double proj = pt.InnerProd(direction_vec);
                if (proj < min_proj)
                {
                    min_proj = proj;
                    *first = pt;
                }
                if (proj > max_proj)
                {
                    max_proj = proj;
                    *last = pt;
                }
            }
        }

        Box2d Polygon2d::BoundingBoxWithHeading(const double heading) const
        {
            CHECK_GE(points_.size(), 3);
            const GPSPoint direction_vec = GPSPoint::CreateUnitVec2d(heading);
            GPSPoint px1;
            GPSPoint px2;
            GPSPoint py1;
            GPSPoint py2;
            ExtremePoints(heading, &px1, &px2);
            ExtremePoints(heading - M_PI_2, &py1, &py2);
            const double x1 = px1.InnerProd(direction_vec);
            const double x2 = px2.InnerProd(direction_vec);
            const double y1 = py1.CrossProd(direction_vec);
            const double y2 = py2.CrossProd(direction_vec);
            GPSPoint center =
                direction_vec * (x1 + x2) / 2.0 +
                GPSPoint(direction_vec.y, -direction_vec.x, 0, 0) * (y1 + y2) / 2.0;
            return Box2d(center, heading, x2 - x1, y2 - y1);
        }

        Box2d Polygon2d::MinAreaBoundingBox() const
        {
            CHECK_GE(points_.size(), 3);
            if (!is_convex_)
            {
                Polygon2d convex_polygon;
                ComputeConvexHull(points_, &convex_polygon);
                CHECK(convex_polygon.is_convex());
                return convex_polygon.MinAreaBoundingBox();
            }
            double min_area = std::numeric_limits<double>::infinity();
            double min_area_at_heading = 0.0;
            int left_most = 0;
            int right_most = 0;
            int top_most = 0;
            for (int i = 0; i < num_points_; ++i)
            {
                const auto &line_segment = line_segments_[i];
                double proj = 0.0;
                double min_proj = line_segment.ProjectOntoUnit(points_[left_most]);
                while ((proj = line_segment.ProjectOntoUnit(points_[Prev(left_most)])) <
                       min_proj)
                {
                    min_proj = proj;
                    left_most = Prev(left_most);
                }
                while ((proj = line_segment.ProjectOntoUnit(points_[Next(left_most)])) <
                       min_proj)
                {
                    min_proj = proj;
                    left_most = Next(left_most);
                }
                double max_proj = line_segment.ProjectOntoUnit(points_[right_most]);
                while ((proj = line_segment.ProjectOntoUnit(points_[Prev(right_most)])) >
                       max_proj)
                {
                    max_proj = proj;
                    right_most = Prev(right_most);
                }
                while ((proj = line_segment.ProjectOntoUnit(points_[Next(right_most)])) >
                       max_proj)
                {
                    max_proj = proj;
                    right_most = Next(right_most);
                }
                double prod = 0.0;
                double max_prod = line_segment.ProductOntoUnit(points_[top_most]);
                while ((prod = line_segment.ProductOntoUnit(points_[Prev(top_most)])) >
                       max_prod)
                {
                    max_prod = prod;
                    top_most = Prev(top_most);
                }
                while ((prod = line_segment.ProductOntoUnit(points_[Next(top_most)])) >
                       max_prod)
                {
                    max_prod = prod;
                    top_most = Next(top_most);
                }
                const double area = max_prod * (max_proj - min_proj);
                if (area < min_area)
                {
                    min_area = area;
                    min_area_at_heading = line_segment.heading();
                }
            }
            return BoundingBoxWithHeading(min_area_at_heading);
        }

        std::vector<LineSegment2d> Polygon2d::GetAllOverlaps(
            const LineSegment2d &line_segment) const
        {
            CHECK_GE(points_.size(), 3);

            if (line_segment.length() <= kMathEpsilon)
            {
                std::vector<LineSegment2d> overlaps;
                if (IsPointIn(line_segment.start()))
                {
                    overlaps.push_back(line_segment);
                }
                return overlaps;
            }
            std::vector<double> projections;
            if (IsPointIn(line_segment.start()))
            {
                projections.push_back(0.0);
            }
            if (IsPointIn(line_segment.end()))
            {
                projections.push_back(line_segment.length());
            }
            for (const auto &poly_seg : line_segments_)
            {
                GPSPoint pt;
                if (poly_seg.GetIntersect(line_segment, &pt))
                {
                    projections.push_back(line_segment.ProjectOntoUnit(pt));
                }
            }
            std::sort(projections.begin(), projections.end());
            std::vector<std::pair<double, double>> overlaps;
            for (size_t i = 0; i + 1 < projections.size(); ++i)
            {
                const double start_proj = projections[i];
                const double end_proj = projections[i + 1];
                if (end_proj - start_proj <= kMathEpsilon)
                {
                    continue;
                }
                const GPSPoint reference_point =
                    line_segment.start() +
                    (line_segment.unit_direction() * (2.0)) / (start_proj + end_proj);
                if (!IsPointIn(reference_point))
                {
                    continue;
                }
                if (overlaps.empty() ||
                    start_proj > overlaps.back().second + kMathEpsilon)
                {
                    overlaps.emplace_back(start_proj, end_proj);
                }
                else
                {
                    overlaps.back().second = end_proj;
                }
            }
            std::vector<LineSegment2d> overlap_line_segments;
            for (const auto &overlap : overlaps)
            {
                overlap_line_segments.emplace_back(
                    line_segment.start() + line_segment.unit_direction() * (overlap.first),
                    line_segment.start() + line_segment.unit_direction() * (overlap.second));
            }
            return overlap_line_segments;
        }

        bool Polygon2d::Contains(const LineSegment2d &line_segment) const
        {
            if (line_segment.length() <= kMathEpsilon)
            {
                return IsPointIn(line_segment.start());
            }
            CHECK_GE(points_.size(), 3);
            if (!IsPointIn(line_segment.start()))
            {
                return false;
            }
            if (!IsPointIn(line_segment.end()))
            {
                return false;
            }
            if (!is_convex_)
            {
                std::vector<LineSegment2d> overlaps = GetAllOverlaps(line_segment);
                double total_length = 0;
                for (const auto &overlap_seg : overlaps)
                {
                    total_length += overlap_seg.length();
                }
                return total_length >= line_segment.length() - kMathEpsilon;
            }
            return true;
        }

        bool Polygon2d::Contains(const Polygon2d &polygon) const
        {
            CHECK_GE(points_.size(), 3);
            if (area_ < polygon.area() - kMathEpsilon)
            {
                return false;
            }
            if (!IsPointIn(polygon.points()[0]))
            {
                return false;
            }
            const auto &line_segments = polygon.line_segments();
            return std::all_of(line_segments.begin(), line_segments.end(),
                               [&](const LineSegment2d &line_segment) {
                                   return Contains(line_segment);
                               });
        }

        int Polygon2d::Next(int at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

        int Polygon2d::Prev(int at) const { return at == 0 ? num_points_ - 1 : at - 1; }

        void Polygon2d::BuildFromPoints()
        {
            num_points_ = points_.size();
            CHECK_GE(num_points_, 3);

            // Make sure the points are in ccw order.
            area_ = 0.0;
            for (int i = 1; i < num_points_; ++i)
            {
                area_ += CrossProd(points_[0], points_[i - 1], points_[i]);
            }
            if (area_ < 0)
            {
                area_ = -area_;
                std::reverse(points_.begin(), points_.end());
            }
            area_ /= 2.0;
            // CHECK_GT(area_, kMathEpsilon);

            // Construct line_segments.
            line_segments_.reserve(num_points_);
            for (int i = 0; i < num_points_; ++i)
            {
                line_segments_.emplace_back(points_[i], points_[Next(i)]);
            }

            // Check convexity.
            is_convex_ = true;
            for (int i = 0; i < num_points_; ++i)
            {
                if (CrossProd(points_[Prev(i)], points_[i], points_[Next(i)]) <=
                    -kMathEpsilon)
                {
                    is_convex_ = false;
                    break;
                }
            }

            // Compute aabox.
            min_x_ = points_[0].x;
            max_x_ = points_[0].x;
            min_y_ = points_[0].y;
            max_y_ = points_[0].y;
            for (const auto &point : points_)
            {
                min_x_ = std::min(min_x_, point.x);
                max_x_ = std::max(max_x_, point.x);
                min_y_ = std::min(min_y_, point.y);
                max_y_ = std::max(max_y_, point.y);
            }
        }

        bool Polygon2d::ComputeConvexHull(const std::vector<GPSPoint> &points,
                                          Polygon2d *const polygon)
        {
            CHECK_NOTNULL(polygon);
            const int n = points.size();
            if (n < 3)
            {
                return false;
            }
            std::vector<int> sorted_indices(n);
            for (int i = 0; i < n; ++i)
            {
                sorted_indices[i] = i;
            }
            std::sort(sorted_indices.begin(), sorted_indices.end(),
                      [&](const int idx1, const int idx2) {
                          const GPSPoint &pt1 = points[idx1];
                          const GPSPoint &pt2 = points[idx2];
                          const double dx = pt1.x - pt2.x;
                          if (std::abs(dx) > kMathEpsilon)
                          {
                              return dx < 0.0;
                          }
                          return pt1.y < pt2.y;
                      });
            int count = 0;
            std::vector<int> results;
            results.reserve(n);
            int last_count = 1;
            for (int i = 0; i < n + n; ++i)
            {
                if (i == n)
                {
                    last_count = count;
                }
                const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
                const GPSPoint &pt = points[idx];
                while (count > last_count &&
                       CrossProd(points[results[count - 2]], points[results[count - 1]],
                                 pt) <= kMathEpsilon)
                {
                    results.pop_back();
                    --count;
                }
                results.push_back(idx);
                ++count;
            }
            --count;
            if (count < 3)
            {
                return false;
            }
            std::vector<GPSPoint> result_points;
            result_points.reserve(count);
            for (int i = 0; i < count; ++i)
            {
                result_points.push_back(points[results[i]]);
            }
            *polygon = Polygon2d(result_points);
            return true;
        }

        bool Polygon2d::ClipConvexHull(const LineSegment2d &line_segment,
                                       std::vector<GPSPoint> *const points)
        {
            if (line_segment.length() <= kMathEpsilon)
            {
                return true;
            }
            CHECK_NOTNULL(points);
            const int n = points->size();
            if (n < 3)
            {
                return false;
            }
            std::vector<double> prod(n);
            std::vector<int> side(n);
            for (int i = 0; i < n; ++i)
            {
                prod[i] = CrossProd(line_segment.start(), line_segment.end(), (*points)[i]);
                if (std::abs(prod[i]) <= kMathEpsilon)
                {
                    side[i] = 0;
                }
                else
                {
                    side[i] = ((prod[i] < 0) ? -1 : 1);
                }
            }

            std::vector<GPSPoint> new_points;
            for (int i = 0; i < n; ++i)
            {
                if (side[i] >= 0)
                {
                    new_points.push_back((*points)[i]);
                }
                const int j = ((i == n - 1) ? 0 : (i + 1));
                if (side[i] * side[j] < 0)
                {
                    const double ratio = prod[j] / (prod[j] - prod[i]);
                    new_points.emplace_back(
                        (*points)[i].x * ratio + (*points)[j].x * (1.0 - ratio),
                        (*points)[i].y * ratio + (*points)[j].y * (1.0 - ratio), 0, 0);
                }
            }

            points->swap(new_points);
            return points->size() >= 3;
        }

        bool Polygon2d::ComputeOverlap(const Polygon2d &other_polygon,
                                       Polygon2d *const overlap_polygon) const
        {
            CHECK_GE(points_.size(), 3);
            CHECK_NOTNULL(overlap_polygon);
            CHECK(is_convex_ && other_polygon.is_convex());
            std::vector<GPSPoint> points = other_polygon.points();
            for (int i = 0; i < num_points_; ++i)
            {
                if (!ClipConvexHull(line_segments_[i], &points))
                {
                    return false;
                }
            }
            return ComputeConvexHull(points, overlap_polygon);
        }

        bool Polygon2d::HasOverlap(const LineSegment2d &line_segment) const
        {
            CHECK_GE(points_.size(), 3);
            if ((line_segment.start().x < min_x_ && line_segment.end().x < min_x_) ||
                (line_segment.start().x > max_x_ && line_segment.end().x > max_x_) ||
                (line_segment.start().y < min_y_ && line_segment.end().y < min_y_) ||
                (line_segment.start().y > max_y_ && line_segment.end().y > max_y_))
            {
                return false;
            }
            GPSPoint first;
            GPSPoint last;
            return GetOverlap(line_segment, &first, &last);
        }

        bool Polygon2d::GetOverlap(const LineSegment2d &line_segment,
                                   GPSPoint *const first, GPSPoint *const last) const
        {
            CHECK_GE(points_.size(), 3);
            CHECK_NOTNULL(first);
            CHECK_NOTNULL(last);

            if (line_segment.length() <= kMathEpsilon)
            {
                if (!IsPointIn(line_segment.start()))
                {
                    return false;
                }
                *first = line_segment.start();
                *last = line_segment.start();
                return true;
            }

            double min_proj = line_segment.length();
            double max_proj = 0;
            if (IsPointIn(line_segment.start()))
            {
                *first = line_segment.start();
                min_proj = 0.0;
            }
            if (IsPointIn(line_segment.end()))
            {
                *last = line_segment.end();
                max_proj = line_segment.length();
            }
            for (const auto &poly_seg : line_segments_)
            {
                GPSPoint pt;
                if (poly_seg.GetIntersect(line_segment, &pt))
                {
                    const double proj = line_segment.ProjectOntoUnit(pt);
                    if (proj < min_proj)
                    {
                        min_proj = proj;
                        *first = pt;
                    }
                    if (proj > max_proj)
                    {
                        max_proj = proj;
                        *last = pt;
                    }
                }
            }
            return min_proj <= max_proj + kMathEpsilon;
        }

        void Polygon2d::GetAllVertices(std::vector<GPSPoint> *const vertices) const
        {
            if (vertices == nullptr)
            {
                return;
            }
            *vertices = points_;
        }

        std::vector<GPSPoint> Polygon2d::GetAllVertices() const
        {
            return points_;
        }

        std::string Polygon2d::DebugString() const
        {
            // return util::StrCat("polygon2d (  num_points = ", num_points_, "  points = (",
            //                     util::PrintDebugStringIter(points_), " )  ",
            //                     is_convex_ ? "convex" : "non-convex", "  area = ", area_,
            //                     " )");
        }

        Box2d::Box2d(const GPSPoint &center, const double heading, const double length,
                     const double width)
            : center_(center),
              length_(length),
              width_(width),
              half_length_(length / 2.0),
              half_width_(width / 2.0),
              heading_(heading),
              cos_heading_(cos(heading)),
              sin_heading_(sin(heading))
        {
            CHECK_GT(length_, -kMathEpsilon);
            CHECK_GT(width_, -kMathEpsilon);
            InitCorners();
        }

        void Box2d::InitCorners()
        {
            const double dx1 = cos_heading_ * half_length_;
            const double dy1 = sin_heading_ * half_length_;
            const double dx2 = sin_heading_ * half_width_;
            const double dy2 = -cos_heading_ * half_width_;
            corners_.clear();
            corners_.emplace_back(center_.x + dx1 + dx2, center_.y + dy1 + dy2, 0, 0);
            corners_.emplace_back(center_.x + dx1 - dx2, center_.y + dy1 - dy2, 0, 0);
            corners_.emplace_back(center_.x - dx1 - dx2, center_.y - dy1 - dy2, 0, 0);
            corners_.emplace_back(center_.x - dx1 + dx2, center_.y - dy1 + dy2, 0, 0);

            for (auto &corner : corners_)
            {
                max_x_ = std::fmax(corner.x, max_x_);
                min_x_ = std::fmin(corner.x, min_x_);
                max_y_ = std::fmax(corner.y, max_y_);
                min_y_ = std::fmin(corner.y, min_y_);
            }
        }

        void Box2d::GetAllCorners(std::vector<GPSPoint> *const corners) const
        {
            if (corners == nullptr)
            {
                return;
            }
            *corners = corners_;
        }

        std::vector<GPSPoint> Box2d::GetAllCorners() const
        {
            return corners_;
        }

        bool Box2d::IsPointIn(const GPSPoint &point) const
        {
            const double x0 = point.x - center_.x;
            const double y0 = point.y - center_.y;
            const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
            const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
            return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
        }

        bool Box2d::IsPointOnBoundary(const GPSPoint &point) const
        {
            const double x0 = point.x - center_.x;
            const double y0 = point.y - center_.y;
            const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
            const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
            return (std::abs(dx - half_length_) <= kMathEpsilon &&
                    dy <= half_width_ + kMathEpsilon) ||
                   (std::abs(dy - half_width_) <= kMathEpsilon &&
                    dx <= half_length_ + kMathEpsilon);
        }

        double Box2d::DistanceTo(const GPSPoint &point) const
        {
            const double x0 = point.x - center_.x;
            const double y0 = point.y - center_.y;
            const double dx =
                std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
            const double dy =
                std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
            if (dx <= 0.0)
            {
                return std::max(0.0, dy);
            }
            if (dy <= 0.0)
            {
                return dx;
            }
            return hypot(dx, dy);
        }

        double Box2d::DistanceTo(const Box2d &box) const
        {
            return Polygon2d(box).DistanceTo(*this);
        }

        bool Box2d::HasOverlap(const Box2d &box) const
        {
            if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
                box.min_y() > max_y())
            {
                return false;
            }

            const double shift_x = box.center_x() - center_.x;
            const double shift_y = box.center_y() - center_.y;

            const double dx1 = cos_heading_ * half_length_;
            const double dy1 = sin_heading_ * half_length_;
            const double dx2 = sin_heading_ * half_width_;
            const double dy2 = -cos_heading_ * half_width_;

            const double dx3 = box.cos_heading() * box.half_length();
            const double dy3 = box.sin_heading() * box.half_length();
            const double dx4 = box.sin_heading() * box.half_width();
            const double dy4 = -box.cos_heading() * box.half_width();

            // const double dx3 = cos_heading_ * half_length_;
            // const double dy3 = sin_heading_ * half_length_;
            // const double dx4 = sin_heading_ * half_width_;
            // const double dy4 = -cos_heading_ * half_width_;
            
            // const double dx1 = box.cos_heading() * box.half_length();
            // const double dy1 = box.sin_heading() * box.half_length();
            // const double dx2 = box.sin_heading() * box.half_width();
            // const double dy2 = -box.cos_heading() * box.half_width();


            return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
                       std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                           std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                           half_length_ &&
                   std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
                       std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                           std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                           half_width_ &&
                   std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
                       std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                           std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                           box.half_length() &&
                   std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
                       std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                           std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                           box.half_width();
        }

        void Box2d::RotateFromCenter(const double rotate_angle)
        {
            heading_ = UtilityHNS::UtilityH::FixNegativeAngle(heading_ + rotate_angle);
            cos_heading_ = std::cos(heading_);
            sin_heading_ = std::sin(heading_);
            InitCorners();
        }

        void Box2d::Shift(const GPSPoint &shift_vec)
        {
            center_ += shift_vec;
            InitCorners();
        }

        void Box2d::LongitudinalExtend(const double extension_length)
        {
            length_ += extension_length;
            half_length_ += extension_length / 2.0;
            InitCorners();
        }

        void Box2d::LateralExtend(const double extension_length)
        {
            width_ += extension_length;
            half_width_ += extension_length / 2.0;
            InitCorners();
        }

        std::string Box2d::DebugString() const
        {
            // return util::StrCat("box2d ( center = ", center_.DebugString(),
            //                     "  heading = ", heading_, "  length = ", length_,
            //                     "  width = ", width_, " )");
        }

        LineSegment2d::LineSegment2d() { unit_direction_ = GPSPoint(1, 0, 0, 0); }

        LineSegment2d::LineSegment2d(const GPSPoint &start, const GPSPoint &end)
            : start_(start), end_(end)
        {
            const double dx = end_.x - start_.x;
            const double dy = end_.y - start_.y;
            length_ = hypot(dx, dy);
            unit_direction_ =
                (length_ <= kMathEpsilon ? GPSPoint(0, 0, 0, 0)
                                         : GPSPoint(dx / length_, dy / length_, 0, 0));
            heading_ = unit_direction_.a;
        }

        double LineSegment2d::length() const { return length_; }

        double LineSegment2d::length_sqr() const { return length_ * length_; }

        double LineSegment2d::DistanceTo(const GPSPoint &point) const
        {
            if (length_ <= kMathEpsilon)
            {
                return point.DistanceTo(start_);
            }
            const double x0 = point.x - start_.x;
            const double y0 = point.y - start_.y;
            const double proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
            if (proj <= 0.0)
            {
                return hypot(x0, y0);
            }
            if (proj >= length_)
            {
                return point.DistanceTo(end_);
            }
            return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
        }

        double LineSegment2d::DistanceTo(const GPSPoint &point,
                                         GPSPoint *const nearest_pt) const
        {
            CHECK_NOTNULL(nearest_pt);

            if (length_ <= kMathEpsilon)
            {
                *nearest_pt = start_;
                return point.DistanceTo(start_);
            }
            const double x0 = point.x - start_.x;
            const double y0 = point.y - start_.y;
            const double proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
            if (proj < 0.0)
            {
                *nearest_pt = start_;
                return hypot(x0, y0);
            }
            if (proj > length_)
            {
                *nearest_pt = end_;
                return point.DistanceTo(end_);
            }
            *nearest_pt = start_ + unit_direction_ * proj;
            return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
        }

        double LineSegment2d::DistanceSquareTo(const GPSPoint &point) const
        {
            if (length_ <= kMathEpsilon)
            {
                return point.DistanceSquareTo(start_);
            }
            const double x0 = point.x - start_.x;
            const double y0 = point.y - start_.y;
            const double proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
            if (proj <= 0.0)
            {
                return Square(x0) + Square(y0);
            }
            if (proj >= length_)
            {
                return point.DistanceSquareTo(end_);
            }
            return Square(x0 * unit_direction_.y - y0 * unit_direction_.x);
        }

        double LineSegment2d::DistanceSquareTo(const GPSPoint &point,
                                               GPSPoint *const nearest_pt) const
        {
            CHECK_NOTNULL(nearest_pt);
            if (length_ <= kMathEpsilon)
            {
                *nearest_pt = start_;
                return point.DistanceSquareTo(start_);
            }
            const double x0 = point.x - start_.x;
            const double y0 = point.y - start_.y;
            const double proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
            if (proj <= 0.0)
            {
                *nearest_pt = start_;
                return Square(x0) + Square(y0);
            }
            if (proj >= length_)
            {
                *nearest_pt = end_;
                return point.DistanceSquareTo(end_);
            }
            *nearest_pt = start_ + unit_direction_ * proj;
            return Square(x0 * unit_direction_.y - y0 * unit_direction_.x);
        }

        bool LineSegment2d::IsPointIn(const GPSPoint &point) const
        {
            if (length_ <= kMathEpsilon)
            {
                return std::abs(point.x - start_.x) <= kMathEpsilon &&
                       std::abs(point.y - start_.y) <= kMathEpsilon;
            }
            const double prod = CrossProd(point, start_, end_);
            if (std::abs(prod) > kMathEpsilon)
            {
                return false;
            }
            return IsWithin(point.x, start_.x, end_.x) &&
                   IsWithin(point.y, start_.y, end_.y);
        }

        double LineSegment2d::ProjectOntoUnit(const GPSPoint &point) const
        {
            return unit_direction_.InnerProd(point - start_);
        }

        double LineSegment2d::ProductOntoUnit(const GPSPoint &point) const
        {
            return unit_direction_.CrossProd(point - start_);
        }

        bool LineSegment2d::HasIntersect(const LineSegment2d &other_segment) const
        {
            GPSPoint point;
            return GetIntersect(other_segment, &point);
        }

        bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                         GPSPoint *const point) const
        {
            CHECK_NOTNULL(point);
            if (IsPointIn(other_segment.start()))
            {
                *point = other_segment.start();
                return true;
            }
            if (IsPointIn(other_segment.end()))
            {
                *point = other_segment.end();
                return true;
            }
            if (other_segment.IsPointIn(start_))
            {
                *point = start_;
                return true;
            }
            if (other_segment.IsPointIn(end_))
            {
                *point = end_;
                return true;
            }
            if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon)
            {
                return false;
            }
            const double cc1 = CrossProd(start_, end_, other_segment.start());
            const double cc2 = CrossProd(start_, end_, other_segment.end());
            if (cc1 * cc2 >= -kMathEpsilon)
            {
                return false;
            }
            const double cc3 =
                CrossProd(other_segment.start(), other_segment.end(), start_);
            const double cc4 =
                CrossProd(other_segment.start(), other_segment.end(), end_);
            if (cc3 * cc4 >= -kMathEpsilon)
            {
                return false;
            }
            const double ratio = cc4 / (cc4 - cc3);
            *point = GPSPoint(start_.x * ratio + end_.x * (1.0 - ratio),
                              start_.y * ratio + end_.y * (1.0 - ratio), 0, 0);
            return true;
        }

        // return distance with perpendicular foot point.
        double LineSegment2d::GetPerpendicularFoot(const GPSPoint &point,
                                                   GPSPoint *const foot_point) const
        {
            CHECK_NOTNULL(foot_point);
            if (length_ <= kMathEpsilon)
            {
                *foot_point = start_;
                return point.DistanceTo(start_);
            }
            const double x0 = point.x - start_.x;
            const double y0 = point.y - start_.y;
            const double proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
            *foot_point = start_ + unit_direction_ * proj;
            return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
        }

        std::string LineSegment2d::DebugString() const
        {
            // return util::StrCat("segment2d ( start = ", start_.DebugString(), "  end = ",
            //                     end_.DebugString(), " )");
        }
    } // namespace math

} // namespace PlannerHNS
