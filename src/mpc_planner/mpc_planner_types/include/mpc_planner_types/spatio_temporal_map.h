#ifndef MPC_PLANNER_TYPES_SPATIO_TEMPORAL_MAP_H
#define MPC_PLANNER_TYPES_SPATIO_TEMPORAL_MAP_H

#include <Eigen/Core>

#include <vector>

namespace MPCPlanner
{
    /**
     * @brief 시공간 맵의 스냅샷을 보관하는 구조체
     *
     * occupancy는 0.0~1.0 확률 값을 저장하며 row-major (x -> y -> z) 순서로 정렬된다.
     * min_x/min_y는 base_link 기준 좌표계에서 첫 셀 중심 좌표를 의미한다.
     */
    struct SpatioTemporalMapSnapshot
    {
        double min_x{0.0};
        double min_y{0.0};

        double resolution_x{0.0};
        double resolution_y{0.0};
        double time_step{0.0};

        int cells_x{0};
        int cells_y{0};
        int layers{0};

        std::vector<float> occupancy;
    };
}

#endif // MPC_PLANNER_TYPES_SPATIO_TEMPORAL_MAP_H

