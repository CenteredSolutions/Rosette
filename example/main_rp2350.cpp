/**
 * @file example/main_rp2350.cpp
 * @brief Minimal RTPS integration example — RP2350 + WizNET W5500 + FreeRTOS.
 *
 * This file demonstrates how to bring up the full RTPS stack on the
 * target platform.  It is intended as a starting point, not a production
 * application.
 *
 * Topology
 * --------
 *   Participant A (this device) publishes a SensorReading on topic
 *   "rt/sensor" every 100 ms using a KEEP_LAST(1) reliable writer.
 *
 *   Participant B (a remote device or a DDS desktop tool such as
 *   rtiddsspy or FastDDS monitor) subscribes to the same topic.
 *
 * Tasks
 * -----
 *   rx_task   — polls the W5500 sockets, feeds raw datagrams to
 *               participant.parse_received().  Single-threaded RX dispatch.
 *
 *   tick_task — calls participant.tick(now_ms) every RTPS_TICK_PERIOD_MS.
 *               Drives SPDP resend, lease expiry, writer HB / retransmit,
 *               and reader deferred ACKNACK.
 *
 *   app_task  — application logic: serialise SensorReading, call
 *               writer->write_reliable().  Blocks inside write_reliable()
 *               until all matched readers acknowledge or the timeout fires.
 *
 * Zero-copy TX path
 * -----------------
 * The example uses buf_alloc_with_headroom() so that RTPS headers are
 * prepended in-place rather than copying the CDR payload into a scratch
 * buffer.  See the write_sensor_reading() function for the full pattern.
 *
 * Platform hooks (must be provided by the BSP — not shown here)
 * -------------------------------------------------------------
 *   uint32_t rtps_platform_get_ms()               — wraps xTaskGetTickCount()
 *   GuidPrefix rtps_platform_make_guid_prefix(id) — MAC-derived prefix
 *   void rtps_platform_flash_erase(addr, size)    — RP2350 flash SDK
 *   void rtps_platform_flash_write(addr, data, sz)
 *   void rtps_platform_flash_read (addr, data, sz)
 */

#include "rtps/rtps.hpp"

// RP2350 / FreeRTOS headers (provided by the SDK and BSP).
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"       // tight_loop_contents(), etc.

// WizNET ioLibrary (provided by the BSP).
#include "wizchip_conf.h"
#include "socket.h"

using namespace rtps;

// ============================================================
// Application data type
// ============================================================

struct SensorReading {
    float    temperature;   // °C
    float    humidity;      // %RH
    uint32_t sequence;
};

// ============================================================
// RTPS entity configuration
// ============================================================

static constexpr uint8_t  kDomainId      = 0;
static constexpr uint8_t  kParticipantId = 0;

// User-defined entity IDs — must not conflict with builtin IDs.
// entityKind 0x02 = user-defined writer (no key), §8.2.4 Table 8.3.
static constexpr EntityId kSensorWriterEid = { {0x00, 0x00, 0x10}, 0x02 };
// entityKind 0x07 = user-defined reader (no key).
static constexpr EntityId kSensorReaderEid = { {0x00, 0x00, 0x10}, 0x07 };

// ============================================================
// Global RTPS objects
// ============================================================

// Transport: metatraffic on port 7410, user data on port 7411.
// SPDP multicast group: 239.255.0.1 (domain 0).
static WiznetUdpTransport g_transport(
    static_cast<uint16_t>(spdp_unicast_port(kDomainId, kParticipantId)),
    static_cast<uint16_t>(7400u + 250u * kDomainId + 1u + 2u * kParticipantId),
    0xEFFF0001u   // 239.255.0.1 in network byte order.
);

static RTPSParticipant* g_participant = nullptr;

// Concrete writer type: KEEP_LAST(1), up to 8 matched readers.
static StatefulWriter<1>* g_sensor_writer = nullptr;

// Application sequence counter.
static uint32_t g_seq = 0;

// ============================================================
// Sensor reading callback — called by StatefulReader when data arrives.
// Runs on the RX dispatch task (rx_task).
// ============================================================

static void on_sensor_reading(const CacheChange& change,
                               void* /*user_data*/) noexcept
{
    if (change.payload.size < sizeof(SensorReading)) return;

    SensorReading reading{};
    memcpy(&reading, change.payload.data, sizeof(reading));

    // In a real application: post to a queue, update a display, etc.
    // Here we just note that the data arrived.
    (void)reading;
}

// ============================================================
// write_sensor_reading() — zero-copy DATA TX path.
//
// Allocates a pool block with headroom for the RTPS + DATA headers,
// serialises the SensorReading CDR directly into the payload region,
// then calls write_reliable() which prepends the headers in-place
// (via BufRef::prepend()) and sends — no scratch buffer, no copy.
// ============================================================

static void write_sensor_reading(float temp, float humidity) noexcept
{
    if (!g_sensor_writer) return;

    // Allocate with kRtpsDataHeadroom reserved before the CDR region.
    // The pool selects the smallest class >= sizeof(SensorReading) + 44.
    BufRef buf = buf_alloc_with_headroom(sizeof(SensorReading),
                                         kRtpsDataHeadroom);
    if (!buf.valid()) return;   // Pool exhausted — drop this sample.

    // Serialise CDR payload directly into the headroom-aware buffer.
    // buf.data points past the reserved header region.
    SensorReading reading{ temp, humidity, ++g_seq };
    memcpy(buf.data, &reading, sizeof(reading));
    buf.size = sizeof(reading);

    // new_change() takes the payload buffer and produces a staging
    // CacheChange.  add_change() stamps the SN and marks it UNSENT.
    BufRef dummy;   // new_change() wants to return the payload ref;
                    // we already have it in buf, so swap it in below.
    CacheChange* ch = g_sensor_writer->new_change(ChangeKind::ALIVE, dummy);
    if (!ch) { buf.reset(); return; }

    // Override the payload with our headroom-allocated block.
    dummy.reset();
    ch->payload = static_cast<BufRef&&>(buf);

    // write_reliable() calls add_change() then blocks until acked or timeout.
    // Because ch->payload.headroom_available() >= kRtpsDataHeadroom,
    // send_data_to() will use the zero-copy prepend path automatically.
    g_sensor_writer->add_change(static_cast<CacheChange&&>(*ch));

    // (In production: call write_reliable() for blocking semantics;
    //  add_change() alone is non-blocking and triggers async TX.)
}

// ============================================================
// RX dispatch task
//
// Polls both W5500 sockets and passes each received datagram to
// the participant's parser.  Single-threaded — no locking needed
// for the on_data / on_heartbeat / on_gap code paths.
// ============================================================

static uint8_t rx_buf[RTPS_POOL_CLASS2_SIZE];   // Stack buffer; not pool-backed.

static void rx_task(void* /*arg*/)
{
    uint32_t src_addr;
    uint16_t src_port;

    for (;;) {
        // Poll metatraffic socket (SPDP, SEDP, HEARTBEAT, ACKNACK).
        uint32_t n = g_transport.poll_meta(rx_buf, sizeof(rx_buf),
                                           src_addr, src_port);
        if (n > 0 && g_participant) {
            g_participant->parse_received(rx_buf, n);
        }

        // Poll user-data socket (DATA, DATA_FRAG, GAP).
        n = g_transport.poll_user(rx_buf, sizeof(rx_buf),
                                  src_addr, src_port);
        if (n > 0 && g_participant) {
            g_participant->parse_received(rx_buf, n);
        }

        // Yield; on a real system with an interrupt-driven receive you
        // would block on a binary semaphore given from the W5500 ISR.
        taskYIELD();
    }
}

// ============================================================
// Tick task
//
// Drives all periodic stack maintenance at RTPS_TICK_PERIOD_MS intervals.
// ============================================================

static void tick_task(void* /*arg*/)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RTPS_TICK_PERIOD_MS));

        if (g_participant) {
            const uint32_t now_ms =
                static_cast<uint32_t>(xTaskGetTickCount() *
                                      portTICK_PERIOD_MS);
            g_participant->tick(now_ms);
        }
    }
}

// ============================================================
// Application task
//
// Publishes a SensorReading every 100 ms.
// ============================================================

static void app_task(void* /*arg*/)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100u));

        // Read sensors (stubbed here).
        const float temp     = 23.5f;
        const float humidity = 55.2f;

        write_sensor_reading(temp, humidity);
    }
}

// ============================================================
// main()
// ============================================================

int main()
{
    // 1. Board / BSP initialisation.
    stdio_init_all();

    // 2. Initialise WizNET W5500 (SPI, reset, etc.) — BSP-specific.
    //    Not shown; the BSP provides wizchip_init() or equivalent.

    // 3. Open transport sockets and join SPDP multicast.
    if (!g_transport.open()) {
        // Fatal — cannot proceed without network.
        for (;;) tight_loop_contents();
    }

    // 4. Construct the participant.
    //    Placement-new into a static buffer avoids heap allocation.
    static alignas(RTPSParticipant)
           uint8_t participant_storage[sizeof(RTPSParticipant)];
    g_participant = new (participant_storage) RTPSParticipant(
        kDomainId, kParticipantId, g_transport);

    // 5. Create a KEEP_LAST(1) reliable writer.
    WriterConfig wcfg{};
    wcfg.entityId   = kSensorWriterEid;
    wcfg.topicName.assign("rt/sensor");
    wcfg.typeName.assign("SensorReading");
    wcfg.reliability= ReliabilityKind::RELIABLE;
    wcfg.durability = DurabilityKind::VOLATILE;

    // Returns StatefulWriter<1>* — depth inferred from template argument.
    g_sensor_writer = g_participant->create_writer<1>(wcfg);

    // 6. Create a matching reader on the same topic (optional on this device;
    //    the remote participant will create its own via SEDP discovery).
    ReaderConfig rcfg{};
    rcfg.entityId   = kSensorReaderEid;
    rcfg.topicName.assign("rt/sensor");
    rcfg.typeName.assign("SensorReading");
    rcfg.reliability= ReliabilityKind::RELIABLE;

    g_participant->create_reader(rcfg, on_sensor_reading, nullptr);

    // 7. Force an immediate SPDP announcement so remote participants
    //    discover us without waiting for the first resend period.
    g_participant->announce();

    // 8. Start FreeRTOS tasks.
    xTaskCreate(rx_task,   "rtps_rx",   512, nullptr, 3, nullptr);
    xTaskCreate(tick_task, "rtps_tick", 256, nullptr, 2, nullptr);
    xTaskCreate(app_task,  "app",       512, nullptr, 1, nullptr);

    // 9. Start the scheduler — does not return.
    vTaskStartScheduler();

    for (;;) tight_loop_contents();
}
