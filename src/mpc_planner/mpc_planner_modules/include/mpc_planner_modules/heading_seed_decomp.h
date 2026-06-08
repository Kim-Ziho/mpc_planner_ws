/**
 * @file heading_seed_decomp.h
 * @brief SeedDecomp variant whose virtual bounding box is aligned with a given heading.
 *
 * decomp_util's SeedDecomp::add_local_bbox() places the virtual walls along the world axes
 * (dir = UnitX, dir_h = UnitY), so the per-seed bounding box is always an axis-parallel square,
 * independent of the robot's direction of travel. EllipsoidDecomp, by contrast, aligns its bbox
 * with the line-segment direction (see line_segment.h / decomp_base.h: "x-axis is parallel to the
 * line, y-axis is perpendicular to the line").
 *
 * A single seed point has no intrinsic direction, so HeadingSeedDecomp takes an explicit heading
 * (yaw) and rotates the virtual walls to match it. This allows an asymmetric corridor that is long
 * in the direction of travel and narrow laterally. local_bbox_(0) is the forward (longitudinal)
 * half-extent, local_bbox_(1) the lateral half-extent — same convention as LineSegment.
 *
 * Only add_local_bbox() is overridden; it stays virtual (DecompBase declares it pure virtual and
 * SeedDecomp overrides it), so both dilate() and set_obs() dispatch to this override — the heading
 * bbox is applied consistently both when building the polyhedron and when pre-filtering obstacle
 * points (points_inside). decomp_util itself is left untouched.
 */
#ifndef __HEADING_SEED_DECOMP_H_
#define __HEADING_SEED_DECOMP_H_

#include <decomp_util/seed_decomp.h>

#include <cmath>

/**
 * @brief Heading-aligned SeedDecomp (2D use only).
 */
template <int Dim>
class HeadingSeedDecomp : public SeedDecomp<Dim>
{
public:
  HeadingSeedDecomp(const Vecf<Dim> &p, double yaw) : SeedDecomp<Dim>(p), yaw_(yaw) {}

protected:
  void add_local_bbox(Polyhedron<Dim> &Vs) override
  {
    if (this->local_bbox_.norm() == 0)
      return;

    // Forward axis (along the heading) and lateral axis (perpendicular), matching LineSegment.
    Vecf<Dim> dir(std::cos(yaw_), std::sin(yaw_));
    Vecf<Dim> dir_h(-std::sin(yaw_), std::cos(yaw_));
    const Vecf<Dim> &p = this->p_;

    // Lateral walls (+/- lateral half-extent = local_bbox_(1)).
    Vs.add(Hyperplane<Dim>(p + dir_h * this->local_bbox_(1), dir_h));
    Vs.add(Hyperplane<Dim>(p - dir_h * this->local_bbox_(1), -dir_h));

    // Longitudinal walls (+/- forward half-extent = local_bbox_(0)).
    Vs.add(Hyperplane<Dim>(p + dir * this->local_bbox_(0), dir));
    Vs.add(Hyperplane<Dim>(p - dir * this->local_bbox_(0), -dir));
  }

  double yaw_;
};

typedef HeadingSeedDecomp<2> HeadingSeedDecomp2D;

#endif // __HEADING_SEED_DECOMP_H_
