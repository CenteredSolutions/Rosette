# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# Ports (ports/<axis>/<name>/): the host chip, POSIX threads, and the host's
# own IP stack through BSD sockets.
set(ROSETTE_CHIP      "host"  PARENT_SCOPE)
set(ROSETTE_OS        "posix" PARENT_SCOPE)
set(ROSETTE_NET_STACK "posix" PARENT_SCOPE)
set(ROSETTE_PROCESSOR "HOST"  PARENT_SCOPE)
