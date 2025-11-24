#include "sample_nodes/tracing.hpp"

#if defined(SAMPLE_NODES_ENABLE_TRACING)
#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE
#include "sample_nodes/tracing.hpp"
#endif
