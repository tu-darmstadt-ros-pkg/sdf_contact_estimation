#ifndef SDF_CONTACT_ESTIMATION_PREDICTION_DIAGNOSTICS_H
#define SDF_CONTACT_ESTIMATION_PREDICTION_DIAGNOSTICS_H

#include <cmath>

namespace sdf_contact_estimation
{

/// Outcome of one pose prediction (SDFContactEstimation::predictPose*).
enum class PredictionStatus : int {
  Ok = 0,              ///< converged to a stable pose with a non-empty final support polygon
  NoSdf = 1,           ///< no SDF loaded
  FellOver = 2,        ///< inclination exceeded the tip-over threshold after a step
  NoHullIteration = 3, ///< no contact (empty support polygon) after a step
  NotConverged = 4,    ///< still unstable after maximum_iterations steps
  NoHullFinal = 5,     ///< stable, but the final contact estimation found no support polygon
  NotPredicted = 6     ///< no prediction done (set by callers for pinned evaluations, and by
                       ///< the estimate*() entry points, which evaluate a given pose only)
};

/// "ok", "no_sdf", "fell_over", "no_hull_iteration", "not_converged", "no_hull_final",
/// "not_predicted"; "unknown" for any other value.
const char *toString( PredictionStatus s );

struct PredictionDiagnostics {
  PredictionStatus status = PredictionStatus::NotPredicted;
  int iterations = 0;                 ///< pose prediction steps done
  double last_step_translation = NAN; ///< [m] pose change of the last iteration
  double last_step_rotation = NAN;    ///< [rad]
  int sampling_points = 0;            ///< queried at the last pose evaluated
  int unknown_sampling_points = 0;    ///< of those, how many touched unobserved voxels
                                      ///< (any interpolation corner NaN / missing block)
};

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_PREDICTION_DIAGNOSTICS_H
