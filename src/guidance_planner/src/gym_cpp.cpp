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
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <fstream>
#include <cstdlib>

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
        // {{2.0, -2.0}, {1.0, 0.0}},  // ped 0: (2, -2) → 위쪽 이동
        // {{5.0, 3.0}, {0.0, -1.0}},  // ped 1: (5,  3) → 아래쪽 이동
        // {{8.5, -1.5}, {-1.0, 0.0}}, // ped 2: (8.5,-1.5) → 위쪽 이동
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

// --- 계획 제어 ---
bool planning_paused_ = false;
bool step_once_ = false;

tf2_ros::TransformBroadcaster *tf_broadcaster_ptr_ = nullptr;

bool pausePlanningCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    planning_paused_ = req.data;
    res.success = true;
    res.message = planning_paused_ ? "Planning paused" : "Planning resumed";
    ROS_INFO("[GymCpp] Planning %s", planning_paused_ ? "PAUSED" : "RESUMED");
    return true;
}

bool stepPlanningCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (!planning_paused_)
    {
        res.success = false;
        res.message = "Not paused. Call /gym_cpp/pause_planning (data: true) first.";
        return true;
    }
    step_once_ = true;
    res.success = true;
    res.message = "Single step will execute on next loop iteration.";
    ROS_INFO("[GymCpp] Step requested.");
    return true;
}

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

// -----------------------------------------------------------------------
// costmap 점유(정적 장애물) 셀 개수 카운트
// LiDAR 스캔이 costmap 을 채웠는지 판단하는 데 사용 (워밍업 대기)
// -----------------------------------------------------------------------
int countOccupiedCells(const costmap_2d::Costmap2D *costmap)
{
    if (costmap == nullptr)
        return 0;
    int count = 0;
    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    for (unsigned int ix = 0; ix < size_x; ++ix)
        for (unsigned int iy = 0; iy < size_y; ++iy)
            if (costmap->getCost(ix, iy) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                ++count;
    return count;
}

// -----------------------------------------------------------------------
// vis_stages 에 따라 시각화할 시간층(gt) 인덱스 선택
// step_map_visualizer.cpp 의 로직과 동일하게 맞춤 (start/terminal 포함)
//  - n<=0 또는 n>=cells_t : 모든 층
//  - n==1                 : 초기 층(gt=0)만
//  - 그 외                : i*(cells_t-1)/(n-1) 반올림으로 균등 분포
// 반환 벡터는 시간 오름차순(첫 원소 = 초기 층).
// -----------------------------------------------------------------------
std::vector<int> selectVisStages(int cells_t, int vis_stages)
{
    std::vector<int> stages;
    int n = vis_stages;
    if (n <= 0 || n >= cells_t)
    {
        for (int gt = 0; gt < cells_t; ++gt)
            stages.push_back(gt);
    }
    else if (n == 1)
    {
        stages.push_back(0);
    }
    else
    {
        for (int i = 0; i < n; ++i)
            stages.push_back(static_cast<int>(std::round(i * (cells_t - 1.0) / (n - 1))));
    }
    return stages;
}

// -----------------------------------------------------------------------
// StepMap 의 선택된 layer들을 CSV로 덤프 후 matplotlib 스크립트로 시각화.
// run_once 단일 실행 시 1회 호출.
// -----------------------------------------------------------------------
void plotStepMapStages(const std::shared_ptr<MPCPlannerStepMap::StepMap> &map,
                       int vis_stages, std::string output_dir, bool show)
{
    if (!map || !map->valid())
    {
        ROS_WARN("[GymCpp] StepMap invalid; skipping plot.");
        return;
    }
    if (output_dir.empty())
        output_dir = ros::package::getPath("guidance_planner");

    const std::string csv_path = output_dir + "/step_map_layers.csv";
    const std::string png_path = output_dir + "/step_map_layers.png";

    const std::vector<int> stages = selectVisStages(map->cellsT(), vis_stages);

    std::ofstream ofs(csv_path);
    if (!ofs.is_open())
    {
        ROS_ERROR("[GymCpp] Failed to open StepMap dump file: %s", csv_path.c_str());
        return;
    }
    ofs << "stage_order,gt,layer_time,gx,gy,x,y,cost\n";
    for (size_t order = 0; order < stages.size(); ++order)
    {
        const int gt = stages[order];
        const double layer_time = map->layerHeight(gt);
        for (int gx = 0; gx < map->cellsX(); ++gx)
        {
            for (int gy = 0; gy < map->cellsY(); ++gy)
            {
                const Eigen::Vector2d w = map->worldFromCell(gx, gy);
                const double cost = map->cellCost(gx, gy, gt);
                ofs << order << ',' << gt << ',' << layer_time << ','
                    << gx << ',' << gy << ','
                    << w.x() << ',' << w.y() << ',' << cost << '\n';
            }
        }
    }
    ofs.close();
    ROS_INFO("[GymCpp] StepMap layers dumped to %s (%zu stages).",
             csv_path.c_str(), stages.size());

    const std::string script = ros::package::getPath("guidance_planner") + "/scripts/plot_step_map.py";
    std::string cmd = "python3 '" + script + "' '" + csv_path + "' --png '" + png_path + "'";
    if (show)
        cmd += " --show";
    cmd += " &"; // 노드를 막지 않도록 백그라운드 실행
    ROS_INFO("[GymCpp] Launching StepMap plot: %s", cmd.c_str());
    if (std::system(cmd.c_str()) != 0)
        ROS_WARN("[GymCpp] Failed to launch plot script (non-zero return).");
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

    // gym 파라미터 (/guidance_planner/gym/* — guidance_planner.yaml 에서 로드)
    //   run_once       : true 면 StepMap + guidance 계획을 한 번만 실행하고,
    //                    이후에는 재계산 없이 시각화/TF 만 유지
    //   plot_step_map  : run_once 완료 시 StepMap layer 분포를 matplotlib 로 시각화
    //   plot_show      : 그래프 창 표시 (false 면 PNG 저장만)
    //   plot_output_dir: CSV/PNG 저장 디렉터리 (빈 문자열이면 guidance_planner 패키지 경로)
    ros::NodeHandle gym_nh("/guidance_planner/gym");
    bool run_once = false;
    gym_nh.param("run_once", run_once, false);
    bool plot_step_map = false;
    gym_nh.param("plot_step_map", plot_step_map, false);
    bool plot_show = true;
    gym_nh.param("plot_show", plot_show, true);
    std::string plot_output_dir;
    gym_nh.param<std::string>("plot_output_dir", plot_output_dir, std::string());

    // vis_stages: step_map 시각화와 동일한 layer 선택 기준
    int vis_stages = 0;
    ros::param::param("/guidance_planner/step_map/vis_stages", vis_stages, 0);

    bool planning_complete_ = false;

    // --- 로봇 상태 구독 ---
    ros::Subscriber robot_state_sub =
        nh.subscribe("/robot_state", 1, robotStateCallback);

    // --- 계획 제어 서비스 ---
    ros::ServiceServer pause_srv =
        nh.advertiseService("pause_planning", pausePlanningCallback);
    ros::ServiceServer step_srv =
        nh.advertiseService("step_planning", stepPlanningCallback);

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
    auto &benchmarker_guidance = BENCHMARKERS.getBenchmarker("Guidance Planning");

    // Reference path 시각화 (메인 루프와 hold 상태에서 재사용)
    auto publishReferencePath = [&]()
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
    };

    // ------------------------------------------------------------------
    // Costmap 워밍업: map→base_link TF 를 발행(robotStateCallback)하면서
    // costmap 이 LiDAR 스캔으로 갱신될 시간을 준다.
    // run_once 단일 실행 시 costmap 이 비어 정적 장애물이 누락되는 것을 방지.
    //   - /robot_state 수신 후 일정 시간(settle) 동안 spin → costmap 다수 업데이트 보장
    //   - 정적 장애물 유무와 무관하게 동작 (장애물 없는 씬에서 타임아웃하지 않음)
    //   - 정적 장애물이 잡히면 그 사실을 로그로만 보고
    // ------------------------------------------------------------------
    {
        ROS_INFO("[GymCpp] Warming up: publishing TF and letting costmap populate...");
        ros::Rate warmup_rate(20.0);
        const int settle_iters_target = 30; // /robot_state 수신 후 ~1.5s @20Hz (costmap 10Hz → 다수 업데이트)
        const int max_iters = 200;          // 최대 ~10s 안전 캡 (/robot_state 미수신 대비)
        int settle = 0;
        int occupied = 0;
        for (int i = 0; i < max_iters && !ros::isShuttingDown(); ++i)
        {
            ros::spinOnce(); // robotStateCallback → map→base_link TF 발행 → costmap 업데이트
            if (robot_state_received_ && ++settle >= settle_iters_target)
            {
                occupied = countOccupiedCells(costmap_ros.getCostmap());
                break;
            }
            warmup_rate.sleep();
        }
        if (!robot_state_received_)
            ROS_WARN("[GymCpp] Warmup ended without /robot_state; will keep waiting in main loop.");
        else if (occupied > 0)
            ROS_INFO("[GymCpp] Costmap ready: %d occupied (static) cells.", occupied);
        else
            ROS_INFO("[GymCpp] Costmap warmed up; no static obstacles detected (dynamic-only scene).");
    }

    ROS_INFO("[GymCpp] Entering main loop (1 Hz)... run_once=%s", run_once ? "true" : "false");
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

        if (planning_paused_ && !step_once_)
        {
            ROS_INFO_THROTTLE(5.0, "[GymCpp] Planning PAUSED. "
                                   "Call /gym_cpp/pause_planning (data: false) or /gym_cpp/step_planning.");
            rate.sleep();
            continue;
        }
        step_once_ = false;

        // run_once: 1회 계획 완료 후에는 재계산 없이 시각화/TF 만 유지
        if (run_once && planning_complete_)
        {
            ROS_INFO_THROTTLE(10.0, "[GymCpp] run_once: planning done. Re-publishing visualization only.");
            guidance.Visualize();
            publishReferencePath();
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
        benchmarker_guidance.start();
        guidance.SetStart(robot_position_, robot_heading_, 0.5 /*speed*/);
        guidance.SetStepMap(step_map);
        guidance.LoadObstacles(pedestrians, static_obstacles);
        guidance.Update();

        benchmarker_guidance.stop();
        benchmarker.stop();

        benchmarker.print();
        benchmarker_guidance.print();

        if (guidance.Succeeded())
        {
            ROS_INFO_STREAM("[GymCpp] Found "
                            << guidance.NumberOfGuidanceTrajectories() << " guidance trajectories.");

            CubicSpline3D &best = guidance.GetGuidanceTrajectory(0).spline;
            RosTools::Spline2D traj = best.GetTrajectory();
            ROS_INFO("[GymCpp] Best trajectory waypoints:");
            for (double t = 0.; t < Config::N * Config::DT; t += Config::DT)
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
        publishReferencePath();

        // run_once 모드: 첫 계획 완료를 표시
        if (run_once)
        {
            if (plot_step_map)
                plotStepMapStages(step_map, vis_stages, plot_output_dir, plot_show);

            planning_complete_ = true;
            ROS_INFO("[GymCpp] run_once enabled: planning executed once. "
                     "Holding (visualization/TF only).");
        }

        rate.sleep();
    }

    BENCHMARKERS.print();
    return 0;
}
