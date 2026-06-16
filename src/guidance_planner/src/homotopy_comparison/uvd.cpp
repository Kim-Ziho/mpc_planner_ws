#include <guidance_planner/homotopy_comparison/uvd.h>

#include <guidance_planner/types/paths.h>
#include <guidance_planner/environment.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace GuidancePlanner
{
    int UVD::ResolveSampleCount(const GeometricPath &a, const GeometricPath &b, Environment &environment) const
    {
        constexpr int kMinSamples = 20;
        constexpr int kMaxSamples = 200;

        if (samples_ > 0) // Fixed sample count configured
            return std::max(samples_, 2);

        // Auto: keep the longitudinal step at most one StepMap cell so the transversal
        // visibility rungs cannot tunnel through a thin band of occupied cells.
        const double resolution = environment.StepMapResolution();
        if (resolution <= 0.0) // No StepMap available -> fall back to the legacy default
            return kMinSamples;

        const double max_length = std::max(a.Length2D(), b.Length2D());
        // +1 for the closing endpoint; halve the resolution for a safety margin.
        const int n = static_cast<int>(std::ceil(max_length / (0.5 * resolution))) + 1;
        return std::clamp(n, kMinSamples, kMaxSamples);
    }

    bool UVD::AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all)
    {
        (void)compute_all;
        /** @note Space-time 3D UVD */
        // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time.
        // The sample count adapts to the StepMap resolution to avoid longitudinal tunneling (see ResolveSampleCount).
        const int num_samples = ResolveSampleCount(a, b, environment);
        Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(num_samples, 0., 1.);

        for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
        {
            if (!environment.IsVisible(a(path_indices(i)), b(path_indices(i))))
                return false;
        }

        return true; // If no collisions occured - paths are homotopic equivalents
    }
} // namespace GuidancePlanner