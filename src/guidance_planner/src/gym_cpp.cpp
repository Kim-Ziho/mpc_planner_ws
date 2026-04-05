/**
 * @file gym_cpp.cpp
 * @brief Gym C++ 노드: Gazebo 2D LiDAR → Costmap2DROS → StepMap → GlobalGuidance
 *
 * jackal_world.launch + test.world 환경에서 SICK LMS1xx front_laser로
 * 정적 장애물을 스캔하여 costmap_2d를 구축하고, GuidancePlanner로 경로 계획 후 RViz 시각화.
 *
 * 실행 방법:
 *   roslaunch guidance_planner ros1_gym_cpp.launch
 */

#include <ros/ros.h>
#include <guidance_planner/global_guidance.h>

#include <ros_tools/profiling.h>
#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>
#include <ros_tools/random_generator.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <mpc_planner_stepmap/step_map_builder.h>
#include <mpc_planner_types/data_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>

using namespace GuidancePlanner;

// -----------------------------------------------------------------------
// 보행자: GuidancePlanner::Obstacle → MPCPlanner::DynamicObstacle 변환
// StepMapBuilder는 DynamicObstacle을 입력받으므로 변환 필요
// -----------------------------------------------------------------------
std::vector<MPCPlanner::DynamicObstacle> toMPCObstacles(const std::vector<GuidancePlanner::Obstacle> &obstacles,
                                                        const std::vector<double> &angles)
{
    std::vector<MPCPlanner::DynamicObstacle> mpc_obstacles;
    mpc_obstacles.reserve(obstacles.size());
    std::vector<double> process_noise = {0.3, 0.3};
    ros::param::get("/pedestrian_simulator/pedestrians/process_noise", process_noise);
    double major_radius = process_noise[0];
    double minor_radius = process_noise[1];

    for (size_t idx = 0; idx < obstacles.size(); ++idx)
    {
        const auto &obs = obstacles[idx];
        const double angle = (idx < angles.size()) ? angles[idx] : 0.0;
        MPCPlanner::DynamicObstacle mpc_obs(obs.id_, obs.positions_[0], angle, obs.radius_);

        MPCPlanner::Prediction pred(MPCPlanner::PredictionType::GAUSSIAN);
        MPCPlanner::Mode mode;
        mode.reserve(obs.positions_.size() - 1);
        for (size_t k = 1; k < obs.positions_.size(); ++k)
            mode.emplace_back(obs.positions_[k], angle, major_radius, minor_radius);
        pred.modes[0] = std::move(mode);
        mpc_obs.prediction = std::move(pred);

        // --- mpc_obs 디버그 출력 ---
        ROS_INFO("[toMPCObstacles] obstacle idx=%zu | id=%d | pos=(%.3f, %.3f) | angle=%.3f rad | radius=%.3f | type=%d",
                 idx, mpc_obs.index,
                 mpc_obs.position.x(), mpc_obs.position.y(),
                 mpc_obs.angle, mpc_obs.radius,
                 static_cast<int>(mpc_obs.type));
        ROS_INFO("  prediction: type=%d | modes=%zu | probabilities=%zu",
                 static_cast<int>(mpc_obs.prediction.type),
                 mpc_obs.prediction.modes.size(),
                 mpc_obs.prediction.probabilities.size());
        for (size_t m = 0; m < mpc_obs.prediction.modes.size(); ++m)
        {
            const auto &mode = mpc_obs.prediction.modes[m];
            double prob = (m < mpc_obs.prediction.probabilities.size()) ? mpc_obs.prediction.probabilities[m] : -1.0;
            ROS_INFO("    mode[%zu]: steps=%zu | prob=%.3f", m, mode.size(), prob);
            for (size_t k = 0; k < mode.size(); ++k)
            {
                const auto &step = mode[k];
                ROS_INFO("      step[%zu]: pos=(%.3f, %.3f) | angle=%.3f | r_maj=%.3f | r_min=%.3f",
                         k, step.position.x(), step.position.y(),
                         step.angle, step.major_radius, step.minor_radius);
            }
        }
        // --------------------------
        mpc_obstacles.push_back(std::move(mpc_obs));
    }
    return mpc_obstacles;
}

// -----------------------------------------------------------------------
// 보행자 샘플: test.world 기준 reference path(x축)를 가로지르는 보행자
// -----------------------------------------------------------------------
void samplePedestrians(std::vector<GuidancePlanner::Obstacle> &obstacles, std::vector<double> &angles)
{
    obstacles.clear();
    angles.clear();

    auto &pub = VISUALS.getPublisher("people");
    auto &model = pub.getNewModelMarker();
    model.setColor(25. / 256., 138. / 256., 89. / 256.);

    auto &line = pub.getNewLine();
    line.setColor(25. / 256., 138. / 256., 89. / 256., 1.0);
    line.setScale(0.1, 0.1);

    // 보행자 정의: {시작위치, 속도벡터}
    // reference path(y=0, x: 0→10)를 가로지르도록 -y→+y 또는 +y→-y 방향으로 이동
    struct PedDef
    {
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
    };
    const std::vector<PedDef> peds = {
        {{3.0, -2.0}, {0.0, 1.0}}, // ped 0: (3, -2) → 위쪽 이동
        {{5.0, 2.0}, {0.0, -1.0}}, // ped 1: (5,  2) → 아래쪽 이동
        {{7.5, -1.5}, {0.0, 1.0}}, // ped 2: (7.5,-1.5) → 위쪽 이동
    };

    for (size_t i = 0; i < peds.size(); ++i)
    {
        const auto &p = peds[i];
        const double angle = std::atan2(p.vel(1), p.vel(0));
        angles.push_back(angle);

        ROS_INFO_STREAM("[GymCpp] Pedestrian " << i
                                               << " angle: " << angle << " rad ("
                                               << angle * 180.0 / M_PI << " deg)");

        obstacles.emplace_back(
            static_cast<int>(i), p.pos, p.vel,
            Config::DT, Config::N, 0.4 /*radius*/);

        // 모델 마커 (사람 아이콘)
        model.setOrientation(
            RosTools::angleToQuaternion(angle + M_PI_2));
        model.addPointMarker(p.pos);

        // 예측 궤적 선
        Eigen::Vector3d cur(p.pos(0), p.pos(1), 0.);
        Eigen::Vector3d prev = cur;
        for (int k = 0; k < Config::N; ++k)
        {
            cur += Eigen::Vector3d(p.vel(0) * Config::DT, p.vel(1) * Config::DT, Config::DT);
            line.addLine(prev, cur);
            prev = cur;
        }
    }
    pub.publish();
}

// --- 로봇 상태 ---
Eigen::Vector2d robot_position_(0., 0.);
double robot_heading_ = 0.;
bool robot_state_received_ = false;

tf2_ros::TransformBroadcaster *tf_broadcaster_ptr_ = nullptr;

void robotStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robot_position_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
    // mobile_robot_state_publisher: orientation.z = yaw
    robot_heading_ = msg->pose.orientation.z;
    robot_state_received_ = true;

    if (tf_broadcaster_ptr_ == nullptr)
        return;

    // map → base_link TF 발행 (costmap_2d::Costmap2DROS 가 필요)
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = "map";
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.translation.x = robot_position_(0);
    tf_stamped.transform.translation.y = robot_position_(1);
    tf_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_heading_);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster_ptr_->sendTransform(tf_stamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gym_cpp");
    ros::NodeHandle nh;
    VISUALS.init(&nh);

    ROS_INFO("[GymCpp] Starting guidance planner gym...");

    // --- TF2 ---
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster_ptr_ = &tf_broadcaster;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // --- Costmap2DROS (2D LiDAR 기반) ---
    // 파라미터: /local_costmap/* (launch 파일에서 로드)
    costmap_2d::Costmap2DROS costmap_ros("local_costmap", tf_buffer);
    costmap_ros.start();

    ROS_INFO("[GymCpp] Waiting for TF chain and first LiDAR scan (2s)...");
    ros::Duration(2.0).sleep();

    // --- StepMapBuilder ---
    ros::NodeHandle nh_private("~");
    MPCPlannerStepMap::StepMapBuilder step_map_builder(nh_private);

    // --- 로봇 상태 구독 ---
    ros::Subscriber robot_state_sub =
        nh.subscribe("/robot_state", 1, robotStateCallback);

    // --- GlobalGuidance 초기화 ---
    ROS_INFO("[GymCpp] Creating GlobalGuidance...");
    GlobalGuidance guidance;
    Config *config = guidance.GetConfig();

    // 레퍼런스 경로: x축 직선 (test.world 배치에 맞춤)
    std::vector<double> ref_xx = {0., 2., 4., 6., 8., 10.};
    std::vector<double> ref_yy = {0., 0., 0., 0., 0., 0.};
    auto reference_path = std::make_shared<RosTools::Spline2D>(ref_xx, ref_yy);
    guidance.LoadReferencePath(0.0, reference_path, 6.0 /*road_width*/);

    // Jackal disc: offset=0, radius=0.325m
    const std::vector<MPCPlanner::Disc> robot_discs = {MPCPlanner::Disc(0.0, 0.325)};

    // 정적 halfspace 없음 (costmap 으로 대체)
    const std::vector<Halfspace> static_obstacles;

    // 보행자 컨테이너
    std::vector<GuidancePlanner::Obstacle> pedestrians;
    std::vector<double> pedestrian_angles;

    auto &benchmarker = BENCHMARKERS.getBenchmarker("GymCpp Planning");

    ROS_INFO("[GymCpp] Entering main loop (1 Hz)...");
    ros::Rate rate(1.0);

    while (!ros::isShuttingDown())
    {
        ros::spinOnce();

        if (!robot_state_received_)
        {
            ROS_WARN_THROTTLE(5.0, "[GymCpp] Waiting for /robot_state...");
            rate.sleep();
            continue;
        }

        ROS_INFO_STREAM("[GymCpp] Robot pose: ("
                        << robot_position_(0) << ", " << robot_position_(1)
                        << ")  heading=" << robot_heading_ << " rad");

        benchmarker.start();

        // 보행자 샘플링 및 시각화
        samplePedestrians(pedestrians, pedestrian_angles);

        // StepMap 생성: costmap(정적) + 보행자(동적)
        auto step_map = step_map_builder.update(
            costmap_ros.getCostmap(),
            robot_position_,
            robot_heading_,
            toMPCObstacles(pedestrians, pedestrian_angles),
            robot_discs,
            Config::N,
            Config::DT);

        // Guidance 계획: 보행자를 동적 장애물로 전달
        guidance.SetStart(robot_position_, robot_heading_, 0.5 /*speed*/);
        guidance.SetStepMap(step_map);
        guidance.LoadObstacles(pedestrians, static_obstacles);
        guidance.Update();

        benchmarker.stop();

        if (guidance.Succeeded())
        {
            ROS_INFO_STREAM("[GymCpp] Found "
                            << guidance.NumberOfGuidanceTrajectories() << " guidance trajectories.");

            CubicSpline3D &best = guidance.GetGuidanceTrajectory(0).spline;
            RosTools::Spline2D traj = best.GetTrajectory();
            ROS_INFO("[GymCpp] Best trajectory waypoints:");
            for (double t = 0.; t < Config::N * Config::DT; t += 4 * Config::DT)
            {
                Eigen::Vector2d pos = traj.getPoint(t);
                ROS_INFO_STREAM("  t=" << std::fixed << std::setprecision(2)
                                       << t << "s -> (" << pos(0) << ", " << pos(1) << ")");
            }
        }
        else
        {
            ROS_WARN("[GymCpp] Guidance found no feasible trajectories.");
        }

        guidance.Visualize();

        // Reference path 시각화
        {
            auto &pub = VISUALS.getPublisher("reference_path");
            auto &line = pub.getNewLine();
            line.setColor(0.2, 0.8, 0.2, 1.0); // 초록색
            line.setScale(0.1, 0.1);

            std::vector<Eigen::Vector2d> pts;
            reference_path->samplePoints(pts, 0.2 /*ds*/);
            for (size_t i = 1; i < pts.size(); ++i)
                line.addLine(Eigen::Vector3d(pts[i - 1](0), pts[i - 1](1), 0.),
                             Eigen::Vector3d(pts[i](0), pts[i](1), 0.));
            pub.publish();
        }

        rate.sleep();
    }

    BENCHMARKERS.print();
    return 0;
}
