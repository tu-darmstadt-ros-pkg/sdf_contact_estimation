#ifndef SDF_CONTACT_ESTIMATION_SDF_QUERY_SCOPE_H
#define SDF_CONTACT_ESTIMATION_SDF_QUERY_SCOPE_H

#include <cstdint>

namespace sdf_contact_estimation
{

namespace detail
{
struct SdfQueryScopeState {
  uint64_t epoch = 0; ///< bumped each time an outermost scope opens on this thread
  int depth = 0;      ///< number of scopes currently open on this thread
};

inline SdfQueryScopeState &sdfQueryScopeState()
{
  thread_local SdfQueryScopeState state;
  return state;
}
} // namespace detail

/// Marks a span of SDF queries on the calling thread during which the map does
/// not change, e.g. one pose prediction. The interpolators reuse looked up
/// blocks only inside the outermost open scope, so nothing cached survives a
/// map update between scopes. Outside any scope every query looks its block up
/// afresh. Scopes nest, only the outermost one starts a new cache epoch.
/// The caller must not modify the map while a scope is open, which any
/// concurrent query would require anyway.
class SdfQueryScope
{
public:
  SdfQueryScope()
  {
    detail::SdfQueryScopeState &state = detail::sdfQueryScopeState();
    if ( state.depth++ == 0 )
      ++state.epoch;
  }

  ~SdfQueryScope() { --detail::sdfQueryScopeState().depth; }

  SdfQueryScope( const SdfQueryScope & ) = delete;
  SdfQueryScope &operator=( const SdfQueryScope & ) = delete;
};

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_SDF_QUERY_SCOPE_H
