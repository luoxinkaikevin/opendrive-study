#pragma once

#include <limits>
#include <string>
#include <vector>
#include "op_utility/UtilityH.h"
#include "op_planner/RoadElement.h"
#include "op_planner/log.h"

namespace PlannerHNS
{
    namespace math
    {
        class Box2d;
        class LineSegment2d;
        /**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
        template <typename T>
        inline T Square(const T value)
        {
            return value * value;
        }

        /**
 * @class Polygon2d
 * @brief The class of polygon in 2-D.
 */
        class Polygon2d
        {
        public:
            Polygon2d() = default;

            explicit Polygon2d(const Box2d &box);
            explicit Polygon2d(std::vector<GPSPoint> points);
            const std::vector<GPSPoint> &points() const { return points_; }
            const std::vector<LineSegment2d> &line_segments() const
            {
                return line_segments_;
            }
            int num_points() const { return num_points_; }
            bool is_convex() const { return is_convex_; }
            double area() const { return area_; }

            /**
   * @brief Compute the distance from a point to the boundary of the polygon.
   *        This distance is equal to the minimal distance from the point
   *        to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon's boundary.
   */
            double DistanceToBoundary(const GPSPoint &point) const;

            /**
   * @brief Compute the distance from a point to the polygon. If the point is
   *        within the polygon, return 0. Otherwise, this distance is
   *        the minimal distance from the point to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon.
   */
            double DistanceTo(const GPSPoint &point) const;

            /**
   * @brief Compute the distance from a line segment to the polygon.
   *        If the line segment is within the polygon, or it has intersect with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance between the distances from the two ends
   *        of the line segment to the polygon.
   * @param line_segment The line segment to compute whose distance to
   *        the polygon.
   * @return The distance from the line segment to the polygon.
   */
            double DistanceTo(const LineSegment2d &line_segment) const;

            /**
   * @brief Compute the distance from a box to the polygon.
   *        If the box is within the polygon, or it has overlap with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the box to the polygon.
   * @param box The box to compute whose distance to the polygon.
   * @return The distance from the box to the polygon.
   */
            double DistanceTo(const Box2d &box) const;

            /**
   * @brief Compute the distance from another polygon to the polygon.
   *        If the other polygon is within this polygon, or it has overlap with
   *        this polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the other polygon to this polygon.
   * @param polygon The polygon to compute whose distance to this polygon.
   * @return The distance from the other polygon to this polygon.
   */
            double DistanceTo(const Polygon2d &polygon) const;

            /**
   * @brief Compute the square of distance from a point to the polygon.
   *        If the point is within the polygon, return 0. Otherwise,
   *        this square of distance is the minimal square of distance from
   *        the point to the edges of the polygon.
   * @param point The point to compute whose square of distance to the polygon.
   * @return The square of distance from the point to the polygon.
   */
            double DistanceSquareTo(const GPSPoint &point) const;

            /**
   * @brief Check if a point is within the polygon.
   * @param point The target point. To check if it is within the polygon.
   * @return Whether a point is within the polygon or not.
   */
            bool IsPointIn(const GPSPoint &point) const;

            /**
   * @brief Check if a point is on the boundary of the polygon.
   * @param point The target point. To check if it is on the boundary
   *        of the polygon.
   * @return Whether a point is on the boundary of the polygon or not.
   */
            bool IsPointOnBoundary(const GPSPoint &point) const;

            /**
   * @brief Check if the polygon contains a line segment.
   * @param line_segment The target line segment. To check if the polygon
   *        contains it.
   * @return Whether the polygon contains the line segment or not.
   */
            bool Contains(const LineSegment2d &line_segment) const;

            /**
   * @brief Check if the polygon contains another polygon.
   * @param polygon The target polygon. To check if this polygon contains it.
   * @return Whether this polygon contains another polygon or not.
   */
            bool Contains(const Polygon2d &polygon) const;

            /**
   * @brief Compute the convex hull of a group of points.
   * @param points The target points. To compute the convex hull of them.
   * @param polygon The convex hull of the points.
   * @return If successfully compute the convex hull.
   */
            static bool ComputeConvexHull(const std::vector<GPSPoint> &points,
                                          Polygon2d *const polygon);

            /**
   * @brief Check if a line segment has overlap with this polygon.
   * @param line_segment The target line segment. To check if it has
   *        overlap with this polygon.
   * @return Whether the target line segment has overlap with this
   *         polygon or not.
   */
            bool HasOverlap(const LineSegment2d &line_segment) const;

            /**
   * @brief Get the overlap of a line segment and this polygon. If they have
   *        overlap, output the two ends of the overlapped line segment.
   * @param line_segment The target line segment. To get its overlap with
   *         this polygon.
   * @param first First end of the overlapped line segment.
   * @param second Second end of the overlapped line segment.
   * @return If the target line segment has overlap with this polygon.
   */
            bool GetOverlap(const LineSegment2d &line_segment, GPSPoint *const first,
                            GPSPoint *const last) const;

            /**
   * @brief Get all vertices of the polygon
   * @param All vertices of the polygon
   */
            void GetAllVertices(std::vector<GPSPoint> *const vertices) const;

            /**
   * @brief Get all vertices of the polygon
   */
            std::vector<GPSPoint> GetAllVertices() const;

            /**
   * @brief Get all overlapped line segments of a line segment and this polygon.
   *        There are possibly multiple overlapped line segments if this
   *        polygon is not convex.
   * @param line_segment The target line segment. To get its all overlapped
   *        line segments with this polygon.
   * @return A group of overlapped line segments.
   */
            std::vector<LineSegment2d> GetAllOverlaps(
                const LineSegment2d &line_segment) const;

            /**
   * @brief Check if this polygon has overlap with another polygon.
   * @param polygon The target polygon. To check if it has overlap
   *        with this polygon.
   * @return If this polygon has overlap with another polygon.
   */
            bool HasOverlap(const Polygon2d &polygon) const;

            // Only compute overlaps between two convex polygons.
            /**
   * @brief Compute the overlap of this polygon and the other polygon if any.
   *        Note: this function only works for computing overlap between
   *        two convex polygons.
   * @param other_polygon The target polygon. To compute its overlap with
   *        this polygon.
   * @param overlap_polygon The overlapped polygon.
   * @param If there is a overlapped polygon.
   */
            bool ComputeOverlap(const Polygon2d &other_polygon,
                                Polygon2d *const overlap_polygon) const;

            /**
   * @brief Get the bound box according to a heading.
   * @param heading The specified heading of the bounding box.
   * @return The bound box according to the specified heading.
   */
            Box2d BoundingBoxWithHeading(const double heading) const;

            /**
   * @brief Get the bounding box with the minimal area.
   * @return The bounding box with the minimal area.
   */
            Box2d MinAreaBoundingBox() const;

            /**
   * @brief Get the extreme points along a heading direction.
   * @param heading The specified heading.
   * @param first The point on the boundary of this polygon with the minimal
   *        projection onto the heading direction.
   * @param last The point on the boundary of this polygon with the maximal
   *        projection onto the heading direction.
   */
            void ExtremePoints(const double heading, GPSPoint *const first,
                               GPSPoint *const last) const;

            /**
   * @brief Expand this polygon by a distance.
   * @param distance The specified distance. To expand this polygon by it.
   * @return The polygon after expansion.
   */
            Polygon2d ExpandByDistance(const double distance) const;

            /**
   * @brief Get a string containing essential information about the polygon
   *        for debugging purpose.
   * @return Essential information about the polygon for debugging purpose.
   */
            std::string DebugString() const;

            double min_x() const { return min_x_; }
            double max_x() const { return max_x_; }
            double min_y() const { return min_y_; }
            double max_y() const { return max_y_; }

        protected:
            void BuildFromPoints();
            int Next(int at) const;
            int Prev(int at) const;

            static bool ClipConvexHull(const LineSegment2d &line_segment,
                                       std::vector<GPSPoint> *const points);

            std::vector<GPSPoint> points_;
            int num_points_ = 0;
            std::vector<LineSegment2d> line_segments_;
            bool is_convex_ = false;
            double area_ = 0.0;
            double min_x_ = 0.0;
            double max_x_ = 0.0;
            double min_y_ = 0.0;
            double max_y_ = 0.0;
        };

        /**
 * @class Box2d
 * @brief Rectangular (undirected) bounding box in 2-D.
 *
 * This class is referential-agnostic, although our convention on the use of
 * the word "heading" in this project (permanently set to be 0 at East)
 * forces us to assume that the X/Y frame here is East/North.
 * For disambiguation, we call the axis of the rectangle parallel to the
 * heading direction the "heading-axis". The size of the heading-axis is
 * called "length", and the size of the axis perpendicular to it "width".
 */
        class Box2d
        {
        public:
            Box2d() = default;
            /**
   * @brief Constructor which takes the center, heading, length and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
            Box2d(const GPSPoint &center, const double heading, const double length,
                  const double width);

            /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
            const GPSPoint &center() const { return center_; }

            /**
   * @brief Getter of the x-coordinate of the center of the box
   * @return The x-coordinate of the center of the box
   */
            double center_x() const { return center_.x; }

            /**
   * @brief Getter of the y-coordinate of the center of the box
   * @return The y-coordinate of the center of the box
   */
            double center_y() const { return center_.y; }

            /**
   * @brief Getter of the length
   * @return The length of the heading-axis
   */
            double length() const { return length_; }

            /**
   * @brief Getter of the width
   * @return The width of the box taken perpendicularly to the heading
   */
            double width() const { return width_; }

            /**
   * @brief Getter of half the length
   * @return Half the length of the heading-axis
   */
            double half_length() const { return half_length_; }

            /**
   * @brief Getter of half the width
   * @return Half the width of the box taken perpendicularly to the heading
   */
            double half_width() const { return half_width_; }

            /**
   * @brief Getter of the heading
   * @return The counter-clockwise angle between the x-axis and the heading-axis
   */
            double heading() const { return heading_; }

            /**
   * @brief Getter of the cosine of the heading
   * @return The cosine of the heading
   */
            double cos_heading() const { return cos_heading_; }

            /**
   * @brief Getter of the sine of the heading
   * @return The sine of the heading
   */
            double sin_heading() const { return sin_heading_; }

            /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
            double area() const { return length_ * width_; }

            /**
   * @brief Getter of the size of the diagonal of the box
   * @return The diagonal size of the box
   */
            double diagonal() const { return std::hypot(length_, width_); }

            /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
            void GetAllCorners(std::vector<GPSPoint> *const corners) const;

            /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
            std::vector<GPSPoint> GetAllCorners() const;

            /**
   * @brief Tests points for membership in the box
   * @param point A point that we wish to test for membership in the box
   * @return True iff the point is contained in the box
   */
            bool IsPointIn(const GPSPoint &point) const;

            /**
   * @brief Tests points for membership in the boundary of the box
   * @param point A point that we wish to test for membership in the boundary
   * @return True iff the point is a boundary point of the box
   */
            bool IsPointOnBoundary(const GPSPoint &point) const;

            /**
   * @brief Determines the distance between the box and a given point
   * @param point The point whose distance to the box we wish to compute
   * @return A distance
   */
            double DistanceTo(const GPSPoint &point) const;

            /**
   * @brief Determines the distance between two boxes
   * @param box The box whose distance to this box we want to compute
   * @return A distance
   */
            double DistanceTo(const Box2d &box) const;

            /**
   * @brief Determines whether these two boxes overlap
   * @param line_segment The other box
   * @return True if they overlap
   */
            bool HasOverlap(const Box2d &box) const;

            /**
   * @brief Rotate from center.
   * @param rotate_angle Angle to rotate.
   */
            void RotateFromCenter(const double rotate_angle);

            /**
   * @brief Shifts this box by a given vector
   * @param shift_vec The vector determining the shift
   */
            void Shift(const GPSPoint &shift_vec);

            /**
   * @brief Extend the box longitudinally
   * @param extension_length the length to extend
   */
            void LongitudinalExtend(const double extension_length);

            void LateralExtend(const double extension_length);

            /**
   * @brief Gets a human-readable description of the box
   * @return A debug-string
   */
            std::string DebugString() const;

            void InitCorners();

            double max_x() const { return max_x_; }
            double min_x() const { return min_x_; }
            double max_y() const { return max_y_; }
            double min_y() const { return min_y_; }

        private:
            GPSPoint center_;
            double length_ = 0.0;
            double width_ = 0.0;
            double half_length_ = 0.0;
            double half_width_ = 0.0;
            double heading_ = 0.0;
            double cos_heading_ = 1.0;
            double sin_heading_ = 0.0;

            std::vector<GPSPoint> corners_;

            double max_x_ = std::numeric_limits<double>::min();
            double min_x_ = std::numeric_limits<double>::max();
            double max_y_ = std::numeric_limits<double>::min();
            double min_y_ = std::numeric_limits<double>::max();
        };

        class LineSegment2d
        {
        public:
            LineSegment2d();

            LineSegment2d(const GPSPoint &start, const GPSPoint &end);
            const GPSPoint &start() const { return start_; }
            const GPSPoint &end() const { return end_; }
            const GPSPoint &unit_direction() const { return unit_direction_; }
            GPSPoint center() const { return (start_ + end_) / 2.0; }
            double heading() const { return heading_; }
            double cos_heading() const { return unit_direction_.x; }
            double sin_heading() const { return unit_direction_.y; }
            double length() const;
            double length_sqr() const;
            double DistanceTo(const GPSPoint &point) const;
            double DistanceTo(const GPSPoint &point, GPSPoint *const nearest_pt) const;
            double DistanceSquareTo(const GPSPoint &point) const;
            double DistanceSquareTo(const GPSPoint &point, GPSPoint *const nearest_pt) const;

            bool IsPointIn(const GPSPoint &point) const;
            bool HasIntersect(const LineSegment2d &other_segment) const;
            bool GetIntersect(const LineSegment2d &other_segment,
                              GPSPoint *const point) const;
            double ProjectOntoUnit(const GPSPoint &point) const;

            double ProductOntoUnit(const GPSPoint &point) const;
            double GetPerpendicularFoot(const GPSPoint &point,
                                        GPSPoint *const foot_point) const;
            std::string DebugString() const;

        private:
            GPSPoint start_;
            GPSPoint end_;
            GPSPoint unit_direction_;
            double heading_ = 0.0;
            double length_ = 0.0;
        };
    } // namespace math

} // namespace PlannerHNS
