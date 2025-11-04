#ifndef MPC_DATA_TYPES_H
#define MPC_DATA_TYPES_H

#include <Eigen/Dense>

#include <vector>
#include <cstddef>

/** Basic high-level data types for motion planning */

namespace MPCPlanner
{

    struct Disc
    {
        double offset;
        double radius;

        Disc(const double offset_, const double radius_);

        Eigen::Vector2d getPosition(const Eigen::Vector2d &robot_position, const double angle) const;
        Eigen::Vector2d toRobotCenter(const Eigen::Vector2d &disc_position, const double angle) const;
    };

    struct Halfspace
    {
        // Ax <= b
        Eigen::Vector2d A;
        double b;

        Halfspace(const Eigen::Vector2d &A, const double b);
    };
    typedef std::vector<Halfspace> StaticObstacle; // For all k, a halfspace

    enum class PredictionType
    {
        DETERMINISTIC = 0,
        GAUSSIAN,
        NONGAUSSIAN,
        NONE
    };

    struct PredictionStep
    {

        // Mean
        Eigen::Vector2d position;
        double angle;

        // Covariance
        double major_radius;
        double minor_radius;

        PredictionStep(const Eigen::Vector2d &position, double angle, double major_radius, double minor_radius);
    };

    typedef std::vector<PredictionStep> Mode;

    struct Prediction
    {

        PredictionType type;

        std::vector<Mode> modes;
        std::vector<double> probabilities;

        Prediction();
        Prediction(PredictionType type);

        bool empty() const;
    };

    enum class ObstacleType
    {
        STATIC = 0,
        DYNAMIC
    };

    struct DynamicObstacle
    {
        int index;

        Eigen::Vector2d position;
        double angle;

        double radius;
        ObstacleType type{ObstacleType::DYNAMIC};

        Prediction prediction;

        DynamicObstacle(int _index, const Eigen::Vector2d &_position, double _angle, double _radius, ObstacleType _type = ObstacleType::DYNAMIC);
    };

    // 시간과 공간을 함께 표현하는 장애물 확률 격자
    struct SpatioTemporalMap
    {
        double resolution_xy{0.0};
        double resolution_t{0.0};
        double origin_x{0.0};
        double origin_y{0.0};
        double origin_t{0.0};

        unsigned int cells_x{0};
        unsigned int cells_y{0};
        unsigned int time_steps{0};

        std::vector<float> data;

        void configure(double resolution_xy_in, double resolution_t_in,
                       double origin_x_in, double origin_y_in, double origin_t_in,
                       unsigned int cells_x_in, unsigned int cells_y_in, unsigned int time_steps_in);
        void clear(float value = 0.f);
        bool empty() const { return data.empty(); }
        bool contains(unsigned int x, unsigned int y, unsigned int t) const
        {
            return x < cells_x && y < cells_y && t < time_steps;
        }

        float &at(unsigned int x, unsigned int y, unsigned int t);
        const float &at(unsigned int x, unsigned int y, unsigned int t) const;

    private:
        size_t index(unsigned int x, unsigned int y, unsigned int t) const;
    };

    struct ReferencePath
    {

        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> psi;

        std::vector<double> v;
        std::vector<double> s;

        ReferencePath(int length = 10);
        void clear();

        bool pointInPath(int point_num, double other_x, double other_y) const;

        bool empty() const { return x.empty(); }
        bool hasVelocity() const { return !v.empty(); }
        bool hasDistance() const { return !s.empty(); }
    };

    typedef ReferencePath Boundary;

    struct Trajectory
    {
        double dt;
        std::vector<Eigen::Vector2d> positions;

        Trajectory(double dt = 0., int length = 10);

        void add(const Eigen::Vector2d &p);
        void add(const double x, const double y);
    };

    struct FixedSizeTrajectory
    {
    private:
        int _size;

    public:
        std::vector<Eigen::Vector2d> positions;

        FixedSizeTrajectory(int size = 50);

        void add(const Eigen::Vector2d &p);
    };
}

#endif
