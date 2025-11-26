#ifndef MPC_PLANNER_STEPMAP_STEP_MAP_H
#define MPC_PLANNER_STEPMAP_STEP_MAP_H

#include <Eigen/Dense>

#include <vector>

namespace MPCPlannerStepMap
{
  /**
   * @brief STEP 맵의 3차원 occupancy grid 데이터 구조
   */
  class StepMap
  {
  public:
    StepMap();

    void configure(int cells_x, int cells_y, int cells_t, double resolution, double time_scale);
    void setPose(const Eigen::Vector2d &center_world, double heading);
    void setOccupancyThreshold(double threshold) { occupancy_threshold_ = threshold; }
    void clear();

    void markStaticWorld(const Eigen::Vector2d &world_point);
    void markDynamicCircleWorld(const Eigen::Vector2d &world_point, int time_index, double radius);
    void addCostWorld(const Eigen::Vector2d &world_point, int time_index, double cost);

    bool isOccupiedWorld(const Eigen::Vector2d &world_point, int time_index) const;
    bool isSegmentOccupiedWorld(const Eigen::Vector2d &start_world, double start_time,
                                const Eigen::Vector2d &end_world, double end_time) const;

    bool valid() const { return cells_x_ > 0 && cells_y_ > 0 && cells_t_ > 0; }

    double resolution() const { return resolution_; }
    int cellsX() const { return cells_x_; }
    int cellsY() const { return cells_y_; }
    int cellsT() const { return cells_t_; }
    double timeScale() const { return time_scale_; }
    double layerHeight(int layer_index) const { return time_scale_ * static_cast<double>(layer_index); }
    double halfLength() const { return half_length_; }
    double halfWidth() const { return half_width_; }

    Eigen::Vector2d worldFromCell(int gx, int gy) const;
    Eigen::Vector2d localFromWorld(const Eigen::Vector2d &world_point) const;
    double cellCost(int gx, int gy, int gt) const;
    bool cellOccupied(int gx, int gy, int gt) const;

  private:
    void markStaticCell(int gx, int gy);
    void markDynamicCell(int gx, int gy, int gt);
    void addCostCell(int gx, int gy, int gt, double cost);

    bool occupiedIndex(int gx, int gy, int gt) const;
    bool insideGrid(int gx, int gy, int gt) const;

    Eigen::Vector3d gridCoordinateFromWorld(const Eigen::Vector2d &world_point, double time_value) const;
    Eigen::Vector2d localFromCell(int gx, int gy) const;
    bool cellFromLocal(const Eigen::Vector2d &local_point, int &gx, int &gy) const;

    size_t idx(int gx, int gy, int gt) const;

  private:
    int cells_x_{0};
    int cells_y_{0};
    int cells_t_{0};

    double resolution_{0.0};
    double half_length_{0.0};
    double half_width_{0.0};
    double time_scale_{1.0};
    double occupancy_threshold_{0.4};

    Eigen::Vector2d center_world_{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d rot_world_from_local_{Eigen::Matrix2d::Identity()};
    Eigen::Matrix2d rot_local_from_world_{Eigen::Matrix2d::Identity()};
    double heading_{0.0};

    std::vector<double> occupancy_;
  };
} // namespace MPCPlannerStepMap

#endif // MPC_PLANNER_STEPMAP_STEP_MAP_H
