/**
 * @file rtps/rtps.hpp
 * @brief Umbrella header — includes all public RTPS stack headers.
 *
 * Application code needs only #include "rosette/rtps/rtps.hpp".
 * Individual headers may still be included directly where
 * compile-time cost is a concern.
 *
 * Delivery order mirrors the spec dependency graph:
 *   types → buf_pool → submessage → history_cache
 *   → message_builder → message_parser
 *   → reader_proxy → writer_proxy
 *   → stateful_writer → stateful_reader
 *   → fragment_assembler
 *   → spdp → endpoint_match_table → sedp
 *   → participant
 *   → transport → persistence
 */

#pragma once

#include "log.hpp"
#include "types.hpp"
#include "buf_pool.hpp"
#include "submessage.hpp"
#include "history_cache.hpp"
#include "message_builder.hpp"
#include "message_parser.hpp"
#include "reader_proxy.hpp"
#include "writer_proxy.hpp"
#include "stateful_writer.hpp"
#include "stateful_reader.hpp"
#include "fragment_assembler.hpp"
#include "spdp.hpp"
#include "endpoint_match_table.hpp"
#include "sedp.hpp"
#include "participant.hpp"
#include "transport.hpp"
#include "persistence.hpp"
