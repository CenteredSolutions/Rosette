# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# MicroPython user module integration for Rosette.
#
# Expected usage:
#   make USER_C_MODULES=/abs/path/to/Rosette/bindings/micropython
#
# The user module points back at the Rosette repository root so the binding can
# include the existing Rosette and micro-ROS headers.

ROSETTE_BINDINGS_DIR := $(USERMOD_DIR)
ROSETTE_REPO_DIR ?= $(abspath $(ROSETTE_BINDINGS_DIR)/../..)
ROSETTE_MICRO_ROS_TRANSPORT ?= udp
ROSETTE_MICRO_ROS_INCLUDE_DIR ?=
ROSETTE_MICRO_ROS_LIBRARY ?=

ROSETTE_MICRO_ROS_CACHE_FILE := $(ROSETTE_REPO_DIR)/build-micro-ros/CMakeCache.txt
ifeq ($(strip $(ROSETTE_MICRO_ROS_INCLUDE_DIR)),)
ifneq ($(wildcard $(ROSETTE_MICRO_ROS_CACHE_FILE)),)
ROSETTE_MICRO_ROS_INCLUDE_DIR := $(strip $(shell sed -n 's/^ROSETTE_MICRO_ROS_INCLUDE_DIR:PATH=//p' "$(ROSETTE_MICRO_ROS_CACHE_FILE)" | head -n 1))
endif
endif
ifeq ($(strip $(ROSETTE_MICRO_ROS_LIBRARY)),)
ifneq ($(wildcard $(ROSETTE_MICRO_ROS_CACHE_FILE)),)
ROSETTE_MICRO_ROS_LIBRARY := $(strip $(shell sed -n 's/^ROSETTE_MICRO_ROS_LIBRARY:FILEPATH=//p' "$(ROSETTE_MICRO_ROS_CACHE_FILE)" | head -n 1))
endif
endif

ROSETTE_MICROPY_HAS_MICRO_ROS := 1

SRC_USERMOD_C += \
	$(ROSETTE_BINDINGS_DIR)/modrosette.c \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_obj_defs.c
SRC_USERMOD_CXX += \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_backend_runtime.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_context.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_node.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_pubsub.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_schema.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_transport.cpp \
	$(ROSETTE_BINDINGS_DIR)/rosette_mpy_stream.cpp

CFLAGS_USERMOD += \
	-DROSETTE_MICROPY_HAS_MICRO_ROS=$(ROSETTE_MICROPY_HAS_MICRO_ROS) \
	-DROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM=$(if $(filter custom,$(ROSETTE_MICRO_ROS_TRANSPORT)),1,0) \
	-I$(ROSETTE_BINDINGS_DIR) \
	-I$(ROSETTE_REPO_DIR) \
	-I$(ROSETTE_REPO_DIR)/include

CXXFLAGS_USERMOD += \
	-std=gnu++17 \
	-DROSETTE_MICROPY_HAS_MICRO_ROS=$(ROSETTE_MICROPY_HAS_MICRO_ROS) \
	-DROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM=$(if $(filter custom,$(ROSETTE_MICRO_ROS_TRANSPORT)),1,0) \
	-I$(ROSETTE_BINDINGS_DIR) \
	-I$(ROSETTE_REPO_DIR) \
	-I$(ROSETTE_REPO_DIR)/include

ifneq ($(ROSETTE_MICROPY_HAS_MICRO_ROS),0)
CFLAGS_USERMOD += -I$(ROSETTE_MICRO_ROS_INCLUDE_DIR)
CXXFLAGS_USERMOD += -I$(ROSETTE_MICRO_ROS_INCLUDE_DIR)
LDFLAGS_USERMOD += $(ROSETTE_MICRO_ROS_LIBRARY)
endif
