#include <sdf_contact_estimation/prediction_diagnostics.h>

namespace sdf_contact_estimation
{

const char *toString( PredictionStatus s )
{
  switch ( s ) {
  case PredictionStatus::Ok:
    return "ok";
  case PredictionStatus::NoSdf:
    return "no_sdf";
  case PredictionStatus::FellOver:
    return "fell_over";
  case PredictionStatus::NoHullIteration:
    return "no_hull_iteration";
  case PredictionStatus::NotConverged:
    return "not_converged";
  case PredictionStatus::NoHullFinal:
    return "no_hull_final";
  case PredictionStatus::NotPredicted:
    return "not_predicted";
  }
  return "unknown";
}

} // namespace sdf_contact_estimation
