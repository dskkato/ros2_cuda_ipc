#pragma once

#include <cstdint>

#define TRACEPOINT_PROVIDER sample_nodes
#define TRACEPOINT_INCLUDE "sample_nodes/tracing.hpp"

#if defined(SAMPLE_NODES_ENABLE_TRACING) &&   \
    (!defined(_SAMPLE_NODES_TRACEPOINTS_H) || \
     defined(TRACEPOINT_HEADER_MULTI_READ))
#define _SAMPLE_NODES_TRACEPOINTS_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    slot_selection,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id, int32_t, result),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id)     //
              ctf_integer(int32_t, result, result))          //
)

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    generation_bump,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id, int32_t, result),
    /* List of fields of eventual event (output) */
    TP_FIELDS(                                         //
        ctf_integer(int32_t, slot_id, slot_id)         //
        ctf_integer(uint64_t, generation, generation)  //
        ctf_integer(uint64_t, byte_size, byte_size)    //
        ctf_integer(int32_t, device_id, device_id)     //
        ctf_integer(int32_t, result, result))          //
)

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    cuda_memset_start,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id))    //
)

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    cuda_memset_stop,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id, int32_t, result),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id)     //
              ctf_integer(int32_t, result, result))          //
)

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    event_record,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id, int32_t, result),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id)     //
              ctf_integer(int32_t, result, result))          //
)

TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    stream_sync,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id, int32_t, result),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id)     //
              ctf_integer(int32_t, result, result))          //
)
TRACEPOINT_EVENT(
    /* Tracepoint provide name */
    sample_nodes,
    /* Tracepoint name */
    message_publication,
    /* List of tracepoint arguments (input) */
    TP_ARGS(int32_t, slot_id, uint64_t, generation, uint64_t, byte_size,
            int32_t, device_id),
    /* List of fields of eventual event (output) */
    TP_FIELDS(ctf_integer(int32_t, slot_id, slot_id)         //
              ctf_integer(uint64_t, generation, generation)  //
              ctf_integer(uint64_t, byte_size, byte_size)    //
              ctf_integer(int32_t, device_id, device_id))    //
)

#endif  // defined(SAMPLE_NODES_ENABLE_TRACING) &&
        // (!defined(_SAMPLE_NODES_TRACEPOINTS_H) ||
        // defined(TRACEPOINT_HEADER_MULTI_READ))

#if defined(SAMPLE_NODES_ENABLE_TRACING)
#include <lttng/tracepoint-event.h>
#endif

#if defined(SAMPLE_NODES_ENABLE_TRACING)
#define SAMPLE_NODES_TRACE_SLOT_SELECTION(slot_id, generation, byte_size,  \
                                          device_id, result)               \
  tracepoint(sample_nodes, slot_selection, slot_id, generation, byte_size, \
             device_id, result)

#define SAMPLE_NODES_TRACE_GENERATION_BUMP(slot_id, generation, byte_size,  \
                                           device_id, result)               \
  tracepoint(sample_nodes, generation_bump, slot_id, generation, byte_size, \
             device_id, result)

#define SAMPLE_NODES_TRACE_CUDA_MEMSET_START(slot_id, generation, byte_size,  \
                                             device_id)                       \
  tracepoint(sample_nodes, cuda_memset_start, slot_id, generation, byte_size, \
             device_id)

#define SAMPLE_NODES_TRACE_CUDA_MEMSET_STOP(slot_id, generation, byte_size,  \
                                            device_id, result)               \
  tracepoint(sample_nodes, cuda_memset_stop, slot_id, generation, byte_size, \
             device_id, result)

#define SAMPLE_NODES_TRACE_EVENT_RECORD(slot_id, generation, byte_size,  \
                                        device_id, result)               \
  tracepoint(sample_nodes, event_record, slot_id, generation, byte_size, \
             device_id, result)

#define SAMPLE_NODES_TRACE_STREAM_SYNC(slot_id, generation, byte_size,  \
                                       device_id, result)               \
  tracepoint(sample_nodes, stream_sync, slot_id, generation, byte_size, \
             device_id, result)

#define SAMPLE_NODES_TRACE_MESSAGE_PUBLICATION(slot_id, generation, byte_size, \
                                               device_id)                      \
  tracepoint(sample_nodes, message_publication, slot_id, generation,           \
             byte_size, device_id)
#else
#define SAMPLE_NODES_TRACE_SLOT_SELECTION(slot_id, generation, byte_size, \
                                          device_id, result)              \
  do {                                                                    \
  } while (false)

#define SAMPLE_NODES_TRACE_GENERATION_BUMP(slot_id, generation, byte_size, \
                                           device_id, result)              \
  do {                                                                     \
  } while (false)

#define SAMPLE_NODES_TRACE_CUDA_MEMSET_START(slot_id, generation, byte_size, \
                                             device_id)                      \
  do {                                                                       \
  } while (false)

#define SAMPLE_NODES_TRACE_CUDA_MEMSET_STOP(slot_id, generation, byte_size, \
                                            device_id, result)              \
  do {                                                                      \
  } while (false)

#define SAMPLE_NODES_TRACE_EVENT_RECORD(slot_id, generation, byte_size, \
                                        device_id, result)              \
  do {                                                                  \
  } while (false)

#define SAMPLE_NODES_TRACE_STREAM_SYNC(slot_id, generation, byte_size, \
                                       device_id, result)              \
  do {                                                                 \
  } while (false)

#define SAMPLE_NODES_TRACE_MESSAGE_PUBLICATION(slot_id, generation, byte_size, \
                                               device_id)                      \
  do {                                                                         \
  } while (false)
#endif
