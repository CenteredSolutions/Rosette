# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# ports/os/stub — the compile-only OS port (rosette_os_port.hpp).
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory. Every target that links `rosette_platform` finds this directory's
# rosette_os_port.hpp through platform.hpp.
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
