#pragma once

#include "message_traits.hpp"

extern "C" {
#include "msgs/sensor_msgs__Imu.h"
}

namespace milkros {

template<>
struct message_traits<sensor_msgs_Imu_t> {
    static constexpr size_t max_serialized_size = 384u;

    static const char* dds_type_name() noexcept {
        return SENSOR_MSGS_IMU_TYPE_NAME;
    }

    static sensor_msgs_Imu_t make_message() noexcept {
        return sensor_msgs_Imu_t{};
    }

    static size_t encode(uint8_t* dst,
                         size_t cap,
                         const sensor_msgs_Imu_t& message) noexcept {
        return encode_sensor_msgs_Imu(dst, cap, &message);
    }

    static bool decode(const uint8_t* src,
                       size_t len,
                       sensor_msgs_Imu_t& message) noexcept {
        return decode_sensor_msgs_Imu(src, len, &message);
    }
};

template<>
inline constexpr bool has_message_traits_v<sensor_msgs_Imu_t> = true;

} // namespace milkros
