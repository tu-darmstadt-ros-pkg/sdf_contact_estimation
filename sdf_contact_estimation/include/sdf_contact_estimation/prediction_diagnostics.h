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
  NotPredicted = 6     ///< no prediction ran (estimate*() calls, or set by callers that skip it)
};

/// Snake case name of s ("ok", "fell_over", ...), "unknown" for a value outside the enum.
const char *toString( PredictionStatus s );

struct PredictionDiagnostics {
  PredictionStatus status = PredictionStatus::NotPredicted;
  int iterations = 0;                 ///< pose prediction steps done
  double last_step_translation = NAN; ///< [m] translation of the last step
  double last_step_rotation = NAN;    ///< [rad] rotation angle of the last step
  int sampling_points = 0;            ///< queried at the last pose evaluated
  int unknown_sampling_points = 0;    ///< of those, the ones whose interpolation used an
                                      ///< unobserved voxel or a missing block
};

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_PREDICTION_DIAGNOSTICS_H
