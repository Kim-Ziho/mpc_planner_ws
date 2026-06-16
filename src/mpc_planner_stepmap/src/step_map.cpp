#include <mpc_planner_stepmap/step_map.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace MPCPlannerStepMap
{
  StepMap::StepMap() = default;

  void StepMap::configure(int cells_x, int cells_y, int cells_t, double resolution, double time_scale)
  {
    cells_x_ = std::max(1, cells_x);
    cells_y_ = std::max(1, cells_y);
    cells_t_ = std::max(1, cells_t);

    resolution_ = resolution;
    time_scale_ = time_scale;

    half_length_ = 0.5 * resolution_ * static_cast<double>(cells_x_);
    half_width_ = 0.5 * resolution_ * static_cast<double>(cells_y_);

    occupancy_.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_) * static_cast<size_t>(cells_t_), 0.0);
  }

  void StepMap::setPose(const Eigen::Vector2d &center_world, double heading)
  {
    heading_ = heading;
    center_world_ = center_world;

    const double c = std::cos(heading_);
    const double s = std::sin(heading_);

    rot_world_from_local_ << c, -s,
        s, c;
    rot_local_from_world_ = rot_world_from_local_.transpose();
  }

  void StepMap::clear()
  {
    std::fill(occupancy_.begin(), occupancy_.end(), 0.0);
  }

  void StepMap::markStaticWorld(const Eigen::Vector2d &world_point)
  {
    Eigen::Vector2d local_point = localFromWorld(world_point);
    int gx, gy;
    if (!cellFromLocal(local_point, gx, gy))
      return;
    markStaticCell(gx, gy);
  }

  void StepMap::markDynamicCircleWorld(const Eigen::Vector2d &world_point, int time_index, double radius)
  {
    if (!valid())
      return;

    Eigen::Vector2d local_point = localFromWorld(world_point);
    int base_x, base_y;
    if (!cellFromLocal(local_point, base_x, base_y))
      return;

    int gt = std::clamp(time_index, 0, cells_t_ - 1);

    int radius_cells = static_cast<int>(std::ceil(radius / resolution_));
    double radius_sq = radius * radius;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        int gx = base_x + dx;
        int gy = base_y + dy;
        if (!insideGrid(gx, gy, gt))
          continue;

        Eigen::Vector2d candidate_local = localFromCell(gx, gy);
        Eigen::Vector2d diff = candidate_local - local_point;
        if (diff.squaredNorm() <= radius_sq)
        {
          markDynamicCell(gx, gy, gt);
        }
      }
    }
  }

  void StepMap::addCostWorld(const Eigen::Vector2d &world_point, int time_index, double cost)
  {
    if (!valid())
      return;

    Eigen::Vector2d local_point = localFromWorld(world_point);
    int gx, gy;
    if (!cellFromLocal(local_point, gx, gy))
      return;
    int gt = std::clamp(time_index, 0, cells_t_ - 1);

    addCostCell(gx, gy, gt, cost);
  }

  bool StepMap::isOccupiedWorld(const Eigen::Vector2d &world_point, int time_index) const
  {
    if (!valid())
      return false;

    Eigen::Vector3d grid_coord = gridCoordinateFromWorld(world_point, static_cast<double>(time_index));
    int gx = static_cast<int>(std::floor(grid_coord.x()));
    int gy = static_cast<int>(std::floor(grid_coord.y()));
    int gt = static_cast<int>(std::floor(grid_coord.z()));

    return occupiedIndex(gx, gy, gt);
  }

  bool StepMap::isSegmentOccupiedWorld(const Eigen::Vector2d &start_world, double start_time,
                                       const Eigen::Vector2d &end_world, double end_time) const
  {
    if (!valid())
      return false;

    auto clipToGrid = [&](const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                          Eigen::Vector3d &clipped_start, Eigen::Vector3d &clipped_end) -> bool {
      constexpr double eps = 1e-6;
      const Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
      const Eigen::Vector3d max_bounds(static_cast<double>(cells_x_) - eps,
                                       static_cast<double>(cells_y_) - eps,
                                       static_cast<double>(cells_t_) - eps);

      double u_enter = 0.0;
      double u_exit = 1.0;
      Eigen::Vector3d delta_local = end - start;

      for (int axis = 0; axis < 3; ++axis)
      {
        const double origin = start[axis];
        const double direction = delta_local[axis];
        const double min_bound = min_bounds[axis];
        const double max_bound = max_bounds[axis];

        if (std::abs(direction) < 1e-9)
        {
          if (origin < min_bound || origin > max_bound)
            return false;
          continue;
        }

        double inv_dir = 1.0 / direction;
        double t0 = (min_bound - origin) * inv_dir;
        double t1 = (max_bound - origin) * inv_dir;
        if (t0 > t1)
          std::swap(t0, t1);

        u_enter = std::max(u_enter, t0);
        u_exit = std::min(u_exit, t1);

        if (u_enter > u_exit)
          return false;
      }

      u_enter = std::clamp(u_enter, 0.0, 1.0);
      u_exit = std::clamp(u_exit, 0.0, 1.0);

      clipped_start = start + u_enter * delta_local;
      clipped_end = start + u_exit * delta_local;
      return true;
    };

    Eigen::Vector3d start_coord = gridCoordinateFromWorld(start_world, start_time);
    Eigen::Vector3d end_coord = gridCoordinateFromWorld(end_world, end_time);
    Eigen::Vector3d clipped_start;
    Eigen::Vector3d clipped_end;

    if (!clipToGrid(start_coord, end_coord, clipped_start, clipped_end))
      return false;

    start_coord = clipped_start;
    end_coord = clipped_end;

    auto clampedIndex = [](double coord, int max_cells) {
      int idx = static_cast<int>(std::floor(coord));
      return std::clamp(idx, 0, max_cells - 1);
    };

    int start_x = clampedIndex(start_coord.x(), cells_x_);
    int start_y = clampedIndex(start_coord.y(), cells_y_);
    int start_t = clampedIndex(start_coord.z(), cells_t_);

    int end_x = clampedIndex(end_coord.x(), cells_x_);
    int end_y = clampedIndex(end_coord.y(), cells_y_);
    int end_t = clampedIndex(end_coord.z(), cells_t_);

    auto isCellOccupied = [&](int gx, int gy, int gt) {
      return insideGrid(gx, gy, gt) && occupancy_[idx(gx, gy, gt)] >= occupancy_threshold_;
    };

    if (isCellOccupied(start_x, start_y, start_t) || isCellOccupied(end_x, end_y, end_t))
      return true;

    Eigen::Vector3d delta = end_coord - start_coord;

    double dir_x = delta.x();
    double dir_y = delta.y();
    double dir_t = delta.z();

    int step_x = (dir_x > 0.0) ? 1 : (dir_x < 0.0 ? -1 : 0);
    int step_y = (dir_y > 0.0) ? 1 : (dir_y < 0.0 ? -1 : 0);
    int step_t = (dir_t > 0.0) ? 1 : (dir_t < 0.0 ? -1 : 0);

    double inv_dx = (step_x == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dir_x);
    double inv_dy = (step_y == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dir_y);
    double inv_dt = (step_t == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dir_t);

    double boundary_x = static_cast<double>(start_x) + (step_x > 0 ? 1.0 : 0.0);
    double boundary_y = static_cast<double>(start_y) + (step_y > 0 ? 1.0 : 0.0);
    double boundary_t = static_cast<double>(start_t) + (step_t > 0 ? 1.0 : 0.0);

    double t_max_x = (step_x == 0) ? std::numeric_limits<double>::infinity()
                                   : (boundary_x - start_coord.x()) / dir_x;
    double t_max_y = (step_y == 0) ? std::numeric_limits<double>::infinity()
                                   : (boundary_y - start_coord.y()) / dir_y;
    double t_max_t = (step_t == 0) ? std::numeric_limits<double>::infinity()
                                   : (boundary_t - start_coord.z()) / dir_t;

    double t_delta_x = (step_x == 0) ? std::numeric_limits<double>::infinity() : inv_dx;
    double t_delta_y = (step_y == 0) ? std::numeric_limits<double>::infinity() : inv_dy;
    double t_delta_t = (step_t == 0) ? std::numeric_limits<double>::infinity() : inv_dt;

    int gx = start_x;
    int gy = start_y;
    int gt = start_t;

    int safety_guard = cells_x_ + cells_y_ + cells_t_ + 10;

    while (safety_guard-- > 0)
    {
      if (isCellOccupied(gx, gy, gt))
        return true;

      if (gx == end_x && gy == end_y && gt == end_t)
        break;

      if (t_max_x <= t_max_y && t_max_x <= t_max_t)
      {
        gx += step_x;
        if (!insideGrid(gx, gy, gt))
          return false;
        t_max_x += t_delta_x;
      }
      else if (t_max_y <= t_max_x && t_max_y <= t_max_t)
      {
        gy += step_y;
        if (!insideGrid(gx, gy, gt))
          return false;
        t_max_y += t_delta_y;
      }
      else
      {
        gt += step_t;
        if (!insideGrid(gx, gy, gt))
          return false;
        t_max_t += t_delta_t;
      }
    }

    return false;
  }

  Eigen::Vector2d StepMap::worldFromCell(int gx, int gy) const
  {
    Eigen::Vector2d local_point = localFromCell(gx, gy);
    return rot_world_from_local_ * local_point + center_world_;
  }

  Eigen::Vector2d StepMap::localFromWorld(const Eigen::Vector2d &world_point) const
  {
    return rot_local_from_world_ * (world_point - center_world_);
  }

  void StepMap::markStaticCell(int gx, int gy)
  {
    if (gx < 0 || gx >= cells_x_ || gy < 0 || gy >= cells_y_)
      return;

    for (int gt = 0; gt < cells_t_; ++gt)
    {
      occupancy_[idx(gx, gy, gt)] = 1.0;
    }
  }

  void StepMap::markDynamicCell(int gx, int gy, int gt)
  {
    if (!insideGrid(gx, gy, gt))
      return;
    occupancy_[idx(gx, gy, gt)] = 1.0;
  }

  void StepMap::addCostCell(int gx, int gy, int gt, double cost)
  {
    if (!insideGrid(gx, gy, gt))
      return;

    size_t index = idx(gx, gy, gt);
    occupancy_[index] = std::clamp(occupancy_[index] + cost, 0.0, 1.0);
  }

  bool StepMap::occupiedIndex(int gx, int gy, int gt) const
  {
    if (!insideGrid(gx, gy, gt))
      return true;
    return occupancy_[idx(gx, gy, gt)] >= occupancy_threshold_;
  }

  bool StepMap::cellOccupied(int gx, int gy, int gt) const
  {
    if (!insideGrid(gx, gy, gt))
      return true;
    return occupancy_[idx(gx, gy, gt)] >= occupancy_threshold_;
  }

  double StepMap::cellCost(int gx, int gy, int gt) const
  {
    if (!insideGrid(gx, gy, gt))
      return 0.0;
    return occupancy_[idx(gx, gy, gt)];
  }

  double StepMap::costWorldInterp(const Eigen::Vector2d &world_point, double time_seconds) const
  {
    if (!valid())
      return 1.0; // 밖=점유 관례 유지

    const double gt_f = (time_scale_ > 1e-9) ? time_seconds / time_scale_ : 0.0;
    const Eigen::Vector3d g = gridCoordinateFromWorld(world_point, gt_f);

    // 셀 중심 보간: occupancy_ 는 셀 중심(연속좌표 정수+0.5) 샘플로 간주하므로 x,y 에
    // half-cell offset 을 준다. 시간축은 layer = time/time_scale 가 곧 정수 샘플(layer
    // height = time_scale·index)이라 offset 없음.
    const double fx = g.x() - 0.5;
    const double fy = g.y() - 0.5;
    const double ft = g.z();

    const int i0 = static_cast<int>(std::floor(fx));
    const int j0 = static_cast<int>(std::floor(fy));
    const int k0 = static_cast<int>(std::floor(ft));
    const double ax = fx - static_cast<double>(i0);
    const double ay = fy - static_cast<double>(j0);
    const double at = ft - static_cast<double>(k0);

    auto sample = [&](int i, int j, int k) -> double {
      if (i < 0 || i >= cells_x_ || j < 0 || j >= cells_y_ || k < 0 || k >= cells_t_)
        return 1.0; // 격자 밖 코너 = 점유
      return occupancy_[idx(i, j, k)];
    };

    // bilinear(x,y) × linear(t)
    auto bilinear = [&](int k) -> double {
      const double c00 = sample(i0, j0, k);
      const double c10 = sample(i0 + 1, j0, k);
      const double c01 = sample(i0, j0 + 1, k);
      const double c11 = sample(i0 + 1, j0 + 1, k);
      const double c0 = c00 * (1.0 - ax) + c10 * ax;
      const double c1 = c01 * (1.0 - ax) + c11 * ax;
      return c0 * (1.0 - ay) + c1 * ay;
    };

    const double b0 = bilinear(k0);
    const double b1 = bilinear(k0 + 1);
    return b0 * (1.0 - at) + b1 * at;
  }

  void StepMap::setCostCell(int gx, int gy, int gt, double cost)
  {
    if (!insideGrid(gx, gy, gt))
      return;
    occupancy_[idx(gx, gy, gt)] = std::clamp(cost, 0.0, 1.0);
  }

  bool StepMap::insideGrid(int gx, int gy, int gt) const
  {
    return gx >= 0 && gx < cells_x_ &&
           gy >= 0 && gy < cells_y_ &&
           gt >= 0 && gt < cells_t_;
  }

  Eigen::Vector3d StepMap::gridCoordinateFromWorld(const Eigen::Vector2d &world_point, double time_value) const
  {
    Eigen::Vector2d local_point = localFromWorld(world_point);
    double gx = (local_point.x() + half_length_) / resolution_;
    double gy = (local_point.y() + half_width_) / resolution_;
    double gt = time_value;
    return Eigen::Vector3d(gx, gy, gt);
  }

  Eigen::Vector2d StepMap::localFromCell(int gx, int gy) const
  {
    double local_x = -half_length_ + (static_cast<double>(gx) + 0.5) * resolution_;
    double local_y = -half_width_ + (static_cast<double>(gy) + 0.5) * resolution_;
    return Eigen::Vector2d(local_x, local_y);
  }

  bool StepMap::cellFromLocal(const Eigen::Vector2d &local_point, int &gx, int &gy) const
  {
    double raw_x = (local_point.x() + half_length_) / resolution_;
    double raw_y = (local_point.y() + half_width_) / resolution_;

    gx = static_cast<int>(std::floor(raw_x));
    gy = static_cast<int>(std::floor(raw_y));

    return gx >= 0 && gx < cells_x_ && gy >= 0 && gy < cells_y_;
  }

  size_t StepMap::idx(int gx, int gy, int gt) const
  {
    size_t x_offset = static_cast<size_t>(gx);
    size_t y_offset = static_cast<size_t>(gy);
    size_t t_offset = static_cast<size_t>(gt);
    return t_offset * static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_) +
           y_offset * static_cast<size_t>(cells_x_) +
           x_offset;
  }
} // namespace MPCPlannerStepMap
