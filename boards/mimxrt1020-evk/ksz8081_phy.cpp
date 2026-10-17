/**
 * @file boards/mimxrt1020-evk/ksz8081_phy.cpp
 * @brief The KSZ8081RNB's RMII clock mode; see ksz8081_phy.hpp.
 */

#include "ksz8081_phy.hpp"

namespace rosette::net::freertos {
namespace {

// ── KSZ8081 vendor registers (KSZ8081RNA/RNB datasheet, register 1Fh) ──
constexpr uint8_t  kKsz8081Control2 = 0x1Fu;
constexpr uint16_t kKsz8081RmiiClock50MHz = 1u << 7;

}  // namespace

bool configure_ksz8081(imxrt::ImxrtEnetLink& link) noexcept {
    uint16_t control2 = 0u;
    return link.phy_read(kKsz8081Control2, control2)
           && link.phy_write(kKsz8081Control2,
                             static_cast<uint16_t>(control2 | kKsz8081RmiiClock50MHz));
}

}  // namespace rosette::net::freertos
