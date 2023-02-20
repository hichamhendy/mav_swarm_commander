#include "mav_swarm_commander/line_segment.h"

LineSegment::LineSegment(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
    : start_(start), end_(end), line_(Eigen::ParametrizedLine<double, 3>::Through(start, end))
{
}

double LineSegment::pointDistance(const Eigen::Vector3d& p) const
{
  const double l = (end_ - start_).norm();

  if (l > 0)
  {
    const auto t = (p - start_).dot(line_.direction()) / l;

    // Start point
    if (t < 0)
    {
      return (p - start_).norm();
    }
    // Point inbetween
    else if (t > 0 && t < 1.0)
    {
      return line_.distance(p);
    }
    // End point
    else
    {
      return (p - end_).norm();
    }
  }
  else
  {
    return (p - end_).norm();
  }
}

double LineSegment::relativePosition(const Eigen::Vector3d& p) const
{
  const double l = (end_ - start_).norm();

  if (l > 0)
  {
    const auto t = (p - start_).dot(line_.direction()) / l;
    return t;
  }
  else
  {
    return 0.0;
  }
}

bool LineSegment::isPointInside(const Eigen::Vector3d& p) const
{
  const double t = relativePosition(p);
  return t >= 0.0 && t <= 1.0;
}

double LineSegment::absOffset(const Eigen::Vector3d& p) const 
{ 
    return line_.distance(p); // the distance of a point p to its projection onto the line *this.
}
