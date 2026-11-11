# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

#[[
rosette_buffer_pool — the one place the RTPS buffer pool's layout is chosen.

The pool (include/rosette/rtps/buf_pool.hpp) has five block classes. A
request takes the smallest class that holds it; a full class promotes to the
next, but never into the large class, which is kept for samples no smaller
class can hold. rmw_rosette (rmw_rosette_capacity.cmake), the direct-core
library (rosette_rtps.cmake), and the firmware-pool spec tests all take
their layout from rosette_buffer_pool_layout() below.

  role       firmware block x count      host block x count   holds
  ---------  --------------------------  -------------------  ----------------------
  control    256 x (8 + HELD_CHANGES)    256 x (32 + E)       ACKNACK, HEARTBEAT,
                                                              GAP; short samples
  discovery  512 x 8 (rebuilt SEDP)      512 x (32 + 2(E+1))  SPDP/SEDP data;
                                                              samples to 512
  medium     1 KiB x 2                   2 KiB x (64 + E)     mid-size samples
  datagram   datagram + 44 hdr x 6       16 KiB x LARGE       firmware: built
                                                              messages, samples
                                                              sent in one datagram
  large      MAX_SAMPLE x LARGE          MAX_SAMPLE x LARGE   larger samples

  E is the local endpoint count; LARGE is the caller's large-sample knob.
  Discovery is 2 x (E + 1) on firmware too when SEDP keeps announcements
  serialized (RTPS_SEDP_REBUILD_ANNOUNCEMENTS=0).

Why the firmware counts: a board with 35 local endpoints among 91 peers
peaked at 7 control, 2 medium, and 2 datagram blocks, so the counts follow
what a board holds at once rather than its endpoint count. Held changes
appear only under loss, usually as small samples, so HELD_CHANGES is added
to control. Datagram blocks leave room for resends and for medium requests
promoted when that class is full. A host datagram reaches 64 KiB, so its
datagram class is a 16 KiB step for mid-size samples.

ROSETTE_POOL_<ROLE>_BLOCKS (CONTROL, DISCOVERY, MEDIUM, DATAGRAM) replaces a
count in any build and profile; measure with ROSETTE_RTPS_POOL_STATS=ON
before lowering one. Every call prints the layout it chose and where each
count came from.
]]

include_guard(GLOBAL)

set(ROSETTE_POOL_CONTROL_BLOCKS "" CACHE STRING
    "Blocks of the buffer pool's control class (256 octets); empty: the profile's count")
set(ROSETTE_POOL_DISCOVERY_BLOCKS "" CACHE STRING
    "Blocks of the buffer pool's discovery class (512 octets); empty: the profile's count")
set(ROSETTE_POOL_MEDIUM_BLOCKS "" CACHE STRING
    "Blocks of the pool's medium class (1 KiB firmware, 2 KiB host); empty: the profile's count")
set(ROSETTE_POOL_DATAGRAM_BLOCKS "" CACHE STRING
    "Blocks of the pool's datagram class (a datagram firmware, 16 KiB host); empty: the profile's")

# Octets of RTPS headers a datagram block keeps ahead of the payload
# (kRtpsDataHeadroom, rosette/rtps/message_builder.hpp).
set(_rbp_rtps_headroom 44)

#[[
Picks one role's block count: the override when its cache entry is set,
otherwise the formula's value clamped to the uint8_t count a class carries.
Sets <out> to the count and <out>_SOURCE to how it was chosen.
]]
function(_rosette_buffer_pool_count out role formula basis)
  set(_override "${ROSETTE_POOL_${role}_BLOCKS}")
  if(NOT _override STREQUAL "")
    if(NOT _override MATCHES "^[0-9]+$" OR _override LESS 1 OR _override GREATER 255)
      message(FATAL_ERROR "ROSETTE_POOL_${role}_BLOCKS (${_override}) must be 1 to 255")
    endif()
    set(${out} ${_override} PARENT_SCOPE)
    set(${out}_SOURCE "ROSETTE_POOL_${role}_BLOCKS" PARENT_SCOPE)
    return()
  endif()

  if(formula GREATER 255)
    set(formula 255)
    set(basis "${basis}, clamped to 255")
  endif()

  set(${out} ${formula} PARENT_SCOPE)
  set(${out}_SOURCE "${basis}" PARENT_SCOPE)
endfunction()

#[[
rosette_buffer_pool_layout(<prefix>
    PROFILE           firmware | host
    MAX_DATAGRAM_SIZE <octets>   largest UDP payload the link carries
    MAX_SAMPLE_SIZE   <octets>   largest serialized sample: the large block
    LARGE_BLOCKS      <count>    large blocks
    LARGE_KNOB        <name>     the cache entry LARGE_BLOCKS came from
    LOCAL_ENDPOINTS   <count>    the larger of local RTPS writers and readers
    HELD_CHANGES      <count>    RTPS_HELD_CHANGES
    SEDP_REBUILD      0 | 1      RTPS_SEDP_REBUILD_ANNOUNCEMENTS
    [LABEL <text>]               names the build in the configure summary
    [QUIET])                     prints no summary

Sets in the caller's scope:
  <prefix>_DEFINITIONS      RTPS_POOL_CLASS<n>_SIZE/_COUNT compile definitions
  <prefix>_DATAGRAM_BLOCKS  the datagram class's count

Fails the configure when the sizes are out of order (each class must be
larger than the one before it, so alloc() can promote) or a count is out of
range.
]]
function(rosette_buffer_pool_layout prefix)
  set(_required PROFILE MAX_DATAGRAM_SIZE MAX_SAMPLE_SIZE LARGE_BLOCKS LARGE_KNOB LOCAL_ENDPOINTS
                HELD_CHANGES SEDP_REBUILD)
  cmake_parse_arguments(PARSE_ARGV 1 arg "QUIET" "${_required};LABEL" "")
  foreach(_required IN LISTS _required)
    if(NOT DEFINED arg_${_required})
      message(FATAL_ERROR "rosette_buffer_pool_layout: ${_required} is required")
    endif()
  endforeach()

  if(NOT arg_LABEL)
    set(arg_LABEL "${prefix}")
  endif()

  if(arg_MAX_SAMPLE_SIZE GREATER 65532)
    message(FATAL_ERROR "${arg_LABEL}: the largest sample (${arg_MAX_SAMPLE_SIZE}) must be at most "
                        "65532 octets (16-bit block sizes)")
  endif()

  # ── discovery count, shared by both profiles ──
  if(arg_SEDP_REBUILD)
    set(_disc_formula 8)
    set(_disc_basis "8 in flight, SEDP rebuilds announcements")
  else()
    math(EXPR _disc_formula "2 * (${arg_LOCAL_ENDPOINTS} + 1)")
    set(_disc_basis "2 x (${arg_LOCAL_ENDPOINTS} endpoints + 1), SEDP keeps announcements")
  endif()

  if(arg_PROFILE STREQUAL "firmware")
    set(_size_control 256)
    set(_size_discovery 512)
    set(_size_medium 1024)
    math(EXPR _size_datagram "((${arg_MAX_DATAGRAM_SIZE} + ${_rbp_rtps_headroom} + 7) / 8) * 8")
    math(EXPR _control "8 + ${arg_HELD_CHANGES}")
    set(_control_basis "8 measured + ${arg_HELD_CHANGES} held changes")
    set(_medium 2)
    set(_medium_basis "2 measured")
    set(_datagram 6)
    set(_datagram_basis "2 measured + 4 for resends")
  elseif(arg_PROFILE STREQUAL "host")
    set(_size_control 256)
    set(_size_discovery 512)
    set(_size_medium 2048)
    set(_size_datagram 16384)
    math(EXPR _control "32 + ${arg_LOCAL_ENDPOINTS}")
    set(_control_basis "32 + ${arg_LOCAL_ENDPOINTS} endpoints")
    math(EXPR _disc_formula "32 + ${_disc_formula}")
    set(_disc_basis "32 + ${_disc_basis}")
    math(EXPR _medium "64 + ${arg_LOCAL_ENDPOINTS}")
    set(_medium_basis "64 + ${arg_LOCAL_ENDPOINTS} endpoints")
    set(_datagram ${arg_LARGE_BLOCKS})
    set(_datagram_basis "${arg_LARGE_KNOB}")
  else()
    message(FATAL_ERROR "${arg_LABEL}: pool profile '${arg_PROFILE}' is not firmware or host")
  endif()

  _rosette_buffer_pool_count(_control CONTROL ${_control} "${_control_basis}")
  _rosette_buffer_pool_count(_discovery DISCOVERY ${_disc_formula} "${_disc_basis}")
  _rosette_buffer_pool_count(_medium MEDIUM ${_medium} "${_medium_basis}")
  _rosette_buffer_pool_count(_datagram DATAGRAM ${_datagram} "${_datagram_basis}")
  if(arg_LARGE_BLOCKS LESS 1 OR arg_LARGE_BLOCKS GREATER 255)
    message(FATAL_ERROR "${arg_LARGE_KNOB} (${arg_LARGE_BLOCKS}) must be 1 to 255")
  endif()

  set(_large ${arg_LARGE_BLOCKS})
  set(_large_SOURCE "${arg_LARGE_KNOB}")
  set(_size_large ${arg_MAX_SAMPLE_SIZE})

  set(_roles control discovery medium datagram large)
  set(_previous 0)
  set(_previous_role "")
  set(_definitions "")
  set(_octets 0)
  set(_class 0)
  set(_rows "")
  foreach(_role IN LISTS _roles)
    if(NOT _size_${_role} GREATER _previous)
      message(FATAL_ERROR
              "${arg_LABEL}: the ${_role} class (${_size_${_role}} octets) must be larger than "
              "the ${_previous_role} class (${_previous} octets) in the ${arg_PROFILE} profile, "
              "so alloc() can promote to it; raise the largest sample size or lower the "
              "datagram size")
    endif()

    set(_previous ${_size_${_role}})
    set(_previous_role ${_role})
    list(APPEND _definitions
         RTPS_POOL_CLASS${_class}_SIZE=${_size_${_role}}u
         RTPS_POOL_CLASS${_class}_COUNT=${_${_role}}u)
    math(EXPR _bytes "${_size_${_role}} * ${_${_role}}")
    math(EXPR _octets "${_octets} + ${_bytes}")
    string(LENGTH "${_role}" _len)
    math(EXPR _pad "10 - ${_len}")
    string(REPEAT " " ${_pad} _gap)
    list(APPEND _rows
         "  ${_role}${_gap}${_size_${_role}} B x ${_${_role}} = ${_bytes} B  (${_${_role}_SOURCE})")
    math(EXPR _class "${_class} + 1")
  endforeach()

  if(NOT arg_QUIET)
    message(STATUS "${arg_LABEL} buffer pool, ${arg_PROFILE} profile: ${_octets} octets of blocks")
    foreach(_row IN LISTS _rows)
      message(STATUS "${_row}")
    endforeach()
  endif()

  set(${prefix}_DEFINITIONS ${_definitions} PARENT_SCOPE)
  set(${prefix}_DATAGRAM_BLOCKS ${_datagram} PARENT_SCOPE)
endfunction()
