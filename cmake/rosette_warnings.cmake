# Compiler warnings for Rosette's own code.
#
#   cmake -S tests/spec -B build/tests-spec -DROSETTE_WERROR=ON
#
# One list of warnings, applied to Rosette's targets and never to third-party
# code (littlefs, micro-ROS, the Pico SDK). Each flag earns its place by what
# goes wrong on a motion controller: narrowing of positions, counts, and
# sequence numbers (-Wconversion), shadowed variables, printf arguments whose
# width differs between the host and the 32-bit target (-Wformat=2), and
# pointer casts that raise alignment, which fault on a Cortex-M0+
# (-Wcast-align).
#
# Style-only flags (-Wold-style-cast, -Wcast-qual, -Wdouble-promotion,
# -Wsign-conversion) are absent: they report no defect and only add casts.
#
# -Wuseless-cast is deliberately absent: a cast that is redundant on a 64-bit
# host is often required on the 32-bit target.
#
# ROSETTE_WERROR makes every warning an error. It is off by default so that a
# newer compiler cannot break a local build; CI turns it on.
#
# Two ways to apply the set:
#   rosette_warnings_for_directory()          every target defined after the call
#   rosette_enable_warnings(<target>...)      the named targets only

option(ROSETTE_WERROR "Treat warnings in Rosette's own code as errors" OFF)

set(ROSETTE_WARNING_FLAGS
    -Wall -Wextra
    -Wconversion
    -Wshadow -Wundef
    -Wcast-align
    -Wnon-virtual-dtor -Woverloaded-virtual
    -Wformat=2)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Only meaningful when optimizing; harmless otherwise.
    list(APPEND ROSETTE_WARNING_FLAGS -Wnull-dereference)
endif()

if (ROSETTE_WERROR)
    list(APPEND ROSETTE_WARNING_FLAGS -Werror)
endif()

# The same flags for C sources, minus the C++-only ones.
set(ROSETTE_C_WARNING_FLAGS ${ROSETTE_WARNING_FLAGS})
list(REMOVE_ITEM ROSETTE_C_WARNING_FLAGS
     -Wnon-virtual-dtor -Woverloaded-virtual)

macro(rosette_warnings_for_directory)
    add_compile_options(
        "$<$<COMPILE_LANGUAGE:CXX>:${ROSETTE_WARNING_FLAGS}>"
        "$<$<COMPILE_LANGUAGE:C>:${ROSETTE_C_WARNING_FLAGS}>")
endmacro()

function(rosette_enable_warnings)
    foreach (_target IN LISTS ARGN)
        get_target_property(_type ${_target} TYPE)
        if (_type STREQUAL "INTERFACE_LIBRARY")
            continue()
        endif()

        target_compile_options(${_target} PRIVATE
            "$<$<COMPILE_LANGUAGE:CXX>:${ROSETTE_WARNING_FLAGS}>"
            "$<$<COMPILE_LANGUAGE:C>:${ROSETTE_C_WARNING_FLAGS}>")
    endforeach()
endfunction()
