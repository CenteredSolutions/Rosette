# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# ports/chip/imxrt/sparse_git_fetch.cmake — FetchContent download step: one
# pinned commit of a git repository, with only the listed paths checked out.
#
# Run as `cmake -P` from a FetchContent_Declare() DOWNLOAD_COMMAND
# (prelude.cmake). A partial (blob-less), depth-1 clone of the release tag
# brings the tree without file contents; the sparse checkout then fetches only
# the files under PATHS, so a vendor SDK of about a gigabyte costs a few
# megabytes. The checkout must resolve to COMMIT, so a moved tag fails the
# configure instead of silently changing the build.
#
# Inputs (-D):
#   REPO    git URL
#   TAG     release tag (or branch) to clone
#   COMMIT  full SHA the tag must resolve to
#   PATHS   '|'-separated sparse-checkout patterns (non-cone, gitignore syntax)
#   DEST    checkout directory; replaced when it holds another commit
#
# Verification: a configure from an empty build directory checks out exactly
# PATHS at COMMIT; changing TAG without COMMIT stops with the SHA mismatch.
#
# References: git-clone(1) --filter=blob:none; git-sparse-checkout(1) --no-cone.

foreach(_var REPO TAG COMMIT PATHS DEST)
  if(NOT DEFINED ${_var} OR "${${_var}}" STREQUAL "")
    message(FATAL_ERROR "sparse_git_fetch.cmake: ${_var} is required")
  endif()
endforeach()

find_package(Git REQUIRED)
string(REPLACE "|" ";" _paths "${PATHS}")

function(_sparse_git)
  execute_process(COMMAND ${GIT_EXECUTABLE} ${ARGN}
                  RESULT_VARIABLE _rc
                  ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "git ${ARGN} failed (${_rc}):\n${_err}")
  endif()
endfunction()

if(EXISTS "${DEST}/.git")
  execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DEST}" rev-parse HEAD
                  OUTPUT_VARIABLE _head OUTPUT_STRIP_TRAILING_WHITESPACE
                  RESULT_VARIABLE _rc ERROR_QUIET)
  if(_rc EQUAL 0 AND _head STREQUAL COMMIT)
    _sparse_git(-C "${DEST}" sparse-checkout set --no-cone ${_paths})
    return()
  endif()
endif()

file(REMOVE_RECURSE "${DEST}")
_sparse_git(clone --filter=blob:none --no-checkout --depth 1 --branch "${TAG}" "${REPO}" "${DEST}")

execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DEST}" rev-parse HEAD
                OUTPUT_VARIABLE _head OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _head STREQUAL COMMIT)
  message(FATAL_ERROR "${REPO} ${TAG} is ${_head}, expected ${COMMIT}")
endif()

_sparse_git(-C "${DEST}" sparse-checkout set --no-cone ${_paths})
_sparse_git(-C "${DEST}" checkout --quiet)
