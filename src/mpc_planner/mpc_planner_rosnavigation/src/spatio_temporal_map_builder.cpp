#include <mpc_planner_rosnavigation/spatio_temporal_map_builder.h>

#include <mpc_planner_types/data_types.h>

#include <ros_tools/logging.h>

#include <ros/time.h>

#include <costmap_2d/cost_values.h>

#include <algorithm>
#include <cmath>

namespace
{
    inline double clampValue(double value, double lower, double upper)
    {
        return std::max(lower, std::min(value, upper));
    }
}

namespace local_planner
{
    SpatioTemporalMapBuilder::SpatioTemporalMapBuilder()
    {
        snapshot_ = std::make_shared<MPCPlanner::SpatioTemporalMapSnapshot>();
    }

    void SpatioTemporalMapBuilder::setParams(const SpatioTemporalMapParams &params)
    {
        params_ = params;
        dimensions_dirty_ = true;
    }

    void SpatioTemporalMapBuilder::setCostmap(const costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
        dimensions_dirty_ = true;
    }

    void SpatioTemporalMapBuilder::update(const Eigen::Vector2d &robot_position,
                                          double robot_yaw,
                                          const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                          double robot_radius,
                                          int horizon_steps)
    {
        if (!params_.enabled)
        {
            snapshot_.reset();
            return;
        }

        if (costmap_ == nullptr)
        {
            LOG_WARN("SpatioTemporalMapBuilder: costmap pointer is null");
            snapshot_.reset();
            return;
        }

        robot_position_ = robot_position;
        robot_yaw_ = robot_yaw;

        refreshDimensions(horizon_steps);

        if (snapshot_ == nullptr)
            return;

        ensureCoordinateCaches();
        clearOccupancy();
        integrateStaticLayer();
        integrateDynamicLayer(dynamic_obstacles, robot_radius, horizon_steps);
    }

    std::shared_ptr<const MPCPlanner::SpatioTemporalMapSnapshot> SpatioTemporalMapBuilder::getSnapshot() const
    {
        return snapshot_;
    }

    visualization_msgs::MarkerArray SpatioTemporalMapBuilder::buildVisualization() const
    {
        visualization_msgs::MarkerArray array;

        if (!snapshot_ || snapshot_->occupancy.empty())
            return array;

        visualization_msgs::Marker marker;
        marker.header.frame_id = params_.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = params_.marker_namespace;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = snapshot_->resolution_x;
        marker.scale.y = snapshot_->resolution_y;
        marker.scale.z = snapshot_->time_step;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.6f;
        marker.color.a = 0.5f;

        const int cells_per_layer = snapshot_->cells_x * snapshot_->cells_y;
        for (int layer_index = 0; layer_index < snapshot_->layers; ++layer_index)
        {
            for (int index_y = 0; index_y < snapshot_->cells_y; ++index_y)
            {
                for (int index_x = 0; index_x < snapshot_->cells_x; ++index_x)
                {
                    const int flat_index = layer_index * cells_per_layer + index_y * snapshot_->cells_x + index_x;
                    const float probability = snapshot_->occupancy[flat_index];
                    if (probability <= 0.0f)
                        continue;

                    geometry_msgs::Point point;
                    point.x = snapshot_->min_x + snapshot_->resolution_x * static_cast<double>(index_x);
                    point.y = snapshot_->min_y + snapshot_->resolution_y * static_cast<double>(index_y);
                    point.z = (static_cast<double>(layer_index) + 0.5) * snapshot_->time_step;
                    marker.points.emplace_back(point);

                    std_msgs::ColorRGBA color;
                    color.r = 1.0f;
                    color.g = 1.0f;
                    color.b = 0.6f;
                    color.a = 0.5f * probability;
                    marker.colors.emplace_back(color);
                }
            }
        }

        if (marker.points.empty())
        {
            marker.action = visualization_msgs::Marker::DELETE;
        }

        array.markers.emplace_back(marker);
        return array;
    }

    void SpatioTemporalMapBuilder::refreshDimensions(int horizon_steps)
    {
        if (costmap_ == nullptr)
            return;

        if (!snapshot_)
            snapshot_ = std::make_shared<MPCPlanner::SpatioTemporalMapSnapshot>();

        const double costmap_resolution = costmap_->getResolution();
        const double costmap_length = costmap_->getSizeInMetersX();
        const double costmap_width = costmap_->getSizeInMetersY();

        const double map_length = costmap_length * params_.length_ratio;
        const double map_width = costmap_width * params_.width_ratio;

        const double resolution_x = std::max(1e-3, costmap_resolution * params_.resolution_ratio_x);
        const double resolution_y = std::max(1e-3, costmap_resolution * params_.resolution_ratio_y);

        const int cells_x = std::max(1, static_cast<int>(std::ceil(map_length / resolution_x)));
        const int cells_y = std::max(1, static_cast<int>(std::ceil(map_width / resolution_y)));
        const int layers = std::max(1, horizon_steps);

        const bool need_resize = dimensions_dirty_ ||
                                 snapshot_->cells_x != cells_x ||
                                 snapshot_->cells_y != cells_y ||
                                 snapshot_->layers != layers ||
                                 std::fabs(snapshot_->resolution_x - resolution_x) > 1e-9 ||
                                 std::fabs(snapshot_->resolution_y - resolution_y) > 1e-9;

        snapshot_->time_step = std::max(params_.time_step, 1e-3);

        if (!need_resize)
            return;

        snapshot_->resolution_x = resolution_x;
        snapshot_->resolution_y = resolution_y;
        snapshot_->cells_x = cells_x;
        snapshot_->cells_y = cells_y;
        snapshot_->layers = layers;

        const double offset = clampValue(params_.heading_offset_ratio, -1.0, 2.0) * map_length;
        min_local_x_ = -0.5 * map_length + offset;
        min_local_y_ = -0.5 * map_width;

        snapshot_->min_x = min_local_x_ + snapshot_->resolution_x * 0.5;
        snapshot_->min_y = min_local_y_ + snapshot_->resolution_y * 0.5;

        const std::size_t total_cells = static_cast<std::size_t>(cells_x) * static_cast<std::size_t>(cells_y) * static_cast<std::size_t>(layers);
        snapshot_->occupancy.assign(total_cells, 0.0f);
        active_indices_.clear();
        cached_center_x_.clear();
        cached_center_y_.clear();

        if (params_.static_cost_threshold < 0.0)
        {
            params_.static_cost_threshold = static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        }

        dimensions_dirty_ = false;
    }

    void SpatioTemporalMapBuilder::clearOccupancy()
    {
        if (!snapshot_)
            return;

        for (const std::size_t flat_index : active_indices_)
        {
            if (flat_index < snapshot_->occupancy.size())
                snapshot_->occupancy[flat_index] = 0.0f;
        }
        active_indices_.clear();
    }

    void SpatioTemporalMapBuilder::integrateStaticLayer()
    {
        if (!snapshot_ || costmap_ == nullptr)
            return;

        const double threshold = params_.static_cost_threshold;
        if (threshold < 0.0)
            return;

        const double cos_yaw = std::cos(robot_yaw_);
        const double sin_yaw = std::sin(robot_yaw_);

        // 정적 장애물은 모든 시간층에 동일하게 반영한다.
        for (int index_x = 0; index_x < snapshot_->cells_x; ++index_x)
        {
            const double local_x = cached_center_x_[index_x];
            for (int index_y = 0; index_y < snapshot_->cells_y; ++index_y)
            {
                const double local_y = cached_center_y_[index_y];

                const double world_x = robot_position_.x() + cos_yaw * local_x - sin_yaw * local_y;
                const double world_y = robot_position_.y() + sin_yaw * local_x + cos_yaw * local_y;

                unsigned int map_x{0};
                unsigned int map_y{0};
                if (!costmap_->worldToMap(world_x, world_y, map_x, map_y))
                    continue;

                const unsigned char cost = costmap_->getCost(map_x, map_y);
                if (static_cast<double>(cost) < threshold)
                    continue;

                for (int layer_index = 0; layer_index < snapshot_->layers; ++layer_index)
                {
                    setOccupied(index_x, index_y, layer_index, 1.0f);
                }
            }
        }
    }

    void SpatioTemporalMapBuilder::integrateDynamicLayer(const std::vector<MPCPlanner::DynamicObstacle> &dynamic_obstacles,
                                                         double robot_radius,
                                                         int horizon_steps)
    {
        if (!snapshot_)
            return;

        const double cos_yaw = std::cos(robot_yaw_);
        const double sin_yaw = std::sin(robot_yaw_);
        const int layers = std::min(snapshot_->layers, std::max(1, horizon_steps));

        for (const auto &obstacle : dynamic_obstacles)
        {
            // 예측이 없을 때는 현재 위치를 계속 사용한다.
            const std::vector<MPCPlanner::PredictionStep> *prediction_steps = nullptr;
            if (!obstacle.prediction.empty() && !obstacle.prediction.modes.empty())
            {
                prediction_steps = &obstacle.prediction.modes.front();
            }

            for (int layer_index = 0; layer_index < layers; ++layer_index)
            {
                Eigen::Vector2d predicted_position = obstacle.position;
                if (layer_index > 0 && prediction_steps != nullptr)
                {
                    const int prediction_index = std::min(layer_index - 1, static_cast<int>(prediction_steps->size()) - 1);
                    if (prediction_index >= 0)
                        predicted_position = (*prediction_steps)[prediction_index].position;
                }

                const double dx = predicted_position.x() - robot_position_.x();
                const double dy = predicted_position.y() - robot_position_.y();

                const double local_x = cos_yaw * dx + sin_yaw * dy;
                const double local_y = -sin_yaw * dx + cos_yaw * dy;

                const double combined_radius = obstacle.radius + robot_radius;
                const double radius_squared = combined_radius * combined_radius;

                const double min_x_local = local_x - combined_radius;
                const double max_x_local = local_x + combined_radius;
                const double min_y_local = local_y - combined_radius;
                const double max_y_local = local_y + combined_radius;

                int index_x_min = static_cast<int>(std::floor((min_x_local - min_local_x_) / snapshot_->resolution_x));
                int index_x_max = static_cast<int>(std::floor((max_x_local - min_local_x_) / snapshot_->resolution_x));
                int index_y_min = static_cast<int>(std::floor((min_y_local - min_local_y_) / snapshot_->resolution_y));
                int index_y_max = static_cast<int>(std::floor((max_y_local - min_local_y_) / snapshot_->resolution_y));

                index_x_min = static_cast<int>(clampValue(static_cast<double>(index_x_min), 0.0, static_cast<double>(snapshot_->cells_x - 1)));
                index_x_max = static_cast<int>(clampValue(static_cast<double>(index_x_max), 0.0, static_cast<double>(snapshot_->cells_x - 1)));
                index_y_min = static_cast<int>(clampValue(static_cast<double>(index_y_min), 0.0, static_cast<double>(snapshot_->cells_y - 1)));
                index_y_max = static_cast<int>(clampValue(static_cast<double>(index_y_max), 0.0, static_cast<double>(snapshot_->cells_y - 1)));

                for (int index_x = index_x_min; index_x <= index_x_max; ++index_x)
                {
                    const double center_x = cached_center_x_[index_x];
                    for (int index_y = index_y_min; index_y <= index_y_max; ++index_y)
                    {
                        const double center_y = cached_center_y_[index_y];
                        const double dx_local = center_x - local_x;
                        const double dy_local = center_y - local_y;
                        const double dist_squared = dx_local * dx_local + dy_local * dy_local;
                        if (dist_squared > radius_squared)
                            continue;

                        setOccupied(index_x, index_y, layer_index, 1.0f);
                    }
                }
            }
        }
    }

    void SpatioTemporalMapBuilder::ensureCoordinateCaches()
    {
        if (!snapshot_)
            return;

        if (cached_center_x_.size() != static_cast<std::size_t>(snapshot_->cells_x))
        {
            cached_center_x_.resize(snapshot_->cells_x);
            for (int index_x = 0; index_x < snapshot_->cells_x; ++index_x)
            {
                cached_center_x_[index_x] = min_local_x_ + snapshot_->resolution_x * (static_cast<double>(index_x) + 0.5);
            }
        }

        if (cached_center_y_.size() != static_cast<std::size_t>(snapshot_->cells_y))
        {
            cached_center_y_.resize(snapshot_->cells_y);
            for (int index_y = 0; index_y < snapshot_->cells_y; ++index_y)
            {
                cached_center_y_[index_y] = min_local_y_ + snapshot_->resolution_y * (static_cast<double>(index_y) + 0.5);
            }
        }
    }

    void SpatioTemporalMapBuilder::setOccupied(int index_x, int index_y, int layer_index, float probability)
    {
        if (!snapshot_)
            return;

        if (index_x < 0 || index_x >= snapshot_->cells_x)
            return;
        if (index_y < 0 || index_y >= snapshot_->cells_y)
            return;
        if (layer_index < 0 || layer_index >= snapshot_->layers)
            return;

        const int cells_per_layer = snapshot_->cells_x * snapshot_->cells_y;
        const std::size_t flat_index = static_cast<std::size_t>(layer_index) * static_cast<std::size_t>(cells_per_layer) +
                                       static_cast<std::size_t>(index_y) * static_cast<std::size_t>(snapshot_->cells_x) +
                                       static_cast<std::size_t>(index_x);

        if (flat_index >= snapshot_->occupancy.size())
            return;

        if (snapshot_->occupancy[flat_index] >= probability)
            return;

        snapshot_->occupancy[flat_index] = probability;
        active_indices_.push_back(flat_index);
    }
}
