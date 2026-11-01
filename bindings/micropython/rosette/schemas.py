# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

"""rosette.schemas — convenience schema constructors for common ROS message types."""

import rosette as _rosette


def temperature_schema():
    """Return a Schema for sensor_msgs/Temperature."""
    return _rosette.schema(
        "sensor_msgs/Temperature",
        "std_msgs/Header header\n"
        "float64 temperature\n"
        "float64 variance\n",
    )
