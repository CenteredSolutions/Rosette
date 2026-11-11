// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_10_4_simulated_writers.cpp
 * @brief Seeded runs of readers sharing a hold pool against simulated writers on a lossy network.
 *
 * tests/formal/RosetteReaderDelivery.tla checks the delivery rules
 * exhaustively, but only at a few changes per writer and only as rules.
 * These tests check the code against the same properties at sizes the model
 * checker cannot reach. Each run builds real SharedPoolStatefulReaders over
 * one HeldChangePool and the real buffer pool, gives each a simulated
 * KEEP_ALL reliable writer, and drives them through a network that loses,
 * duplicates and reorders messages, chosen by a seeded generator so any
 * failure replays from its seed.
 *
 * A run has a lossy phase and then a fair phase, in which nothing is lost
 * and every message in flight is eventually received. The oracles mirror
 * the model's properties:
 *  - InOrder: each reader delivers exactly its next change, never a later
 *    one and never one twice (checked at every delivery).
 *  - NoAckPastUndelivered: no ACKNACK acknowledges a change its reader has
 *    not delivered (checked at every ACKNACK).
 *  - WithinCap: the pool never holds more than its slots.
 *  - EveryChangeDone: by the end of the fair phase every reader whose writer
 *    kept going has delivered every change.
 *  - No leak: once the readers are gone, every buffer block is free again.
 *
 * ROSETTE_SIM_SEEDS sets the number of seeds per test (default 200), for a
 * longer campaign; ROSETTE_SIM_SEED runs one seed, to replay a failure.
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "spec_test_common.hpp"

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

extern uint32_t g_stateful_reader_test_now_ms;

namespace {

/// Sample size: it lands in the block class MessageBuilder takes for an
/// ACKNACK (kControlMessageCapacity), so samples and ACKNACKs compete.
constexpr uint32_t kSampleSize = 200u;
static_assert(kSampleSize <= MessageBuilder::kControlMessageCapacity
                  && kSampleSize > RTPS_POOL_CLASS0_SIZE,
              "samples share the ACKNACK's block class");

/// SplitMix64: small, fast, and the same sequence on every platform.
struct Rng {
    uint64_t state;

    uint64_t next() {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }

    bool percent(uint32_t p) { return below(100u) < p; }
};

/// One run's shape; the seed picks everything else.
struct SimConfig {
    uint8_t  readers;      ///< 1 to 4, each with its own writer.
    uint8_t  limit;        ///< Hold slots one reader may occupy (its window).
    int64_t  changes;      ///< Changes each writer writes.
    uint32_t loss_pct;     ///< Chance a message is lost during the lossy phase.
    uint32_t lossy_steps;  ///< Steps before the network turns fair.
    uint8_t spare_blocks;  ///< Blocks a small sample may use left free; 0 leaves the pool as built.
    bool    silent;        ///< Writer 0 stops for good at a random step.
};

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> sent{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// A message from a simulated writer to its reader.
struct Msg {
    bool    heartbeat;
    uint8_t to;
    int64_t sn;    ///< DATA: the change. HEARTBEAT: firstSN.
    int64_t last;  ///< HEARTBEAT: lastSN.
};

/// A KEEP_ALL reliable writer and what its reader delivered.
struct SimWriter {
    Guid                                      reader{};
    Guid                                      writer{};
    int64_t                                   written = 0;
    int64_t                                   acked = 0;
    bool                                      alive = true;
    int32_t                                   hb_count = 0;
    int64_t                                   delivered = 0;
    std::string*                              failure = nullptr;
    std::unique_ptr<SharedPoolStatefulReader> r{};

    /// InOrder: the only change a reader may deliver is the next one.
    static void on_change(const CacheChange& c, void* user) noexcept {
        SimWriter*    self = static_cast<SimWriter*>(user);
        const int64_t sn = c.sequenceNumber.to_int64();
        if (sn != self->delivered + 1 && self->failure->empty()) {
            *self->failure = "InOrder: delivered SN " + std::to_string(sn) + " after "
                             + std::to_string(self->delivered);
        }

        self->delivered = sn;
    }

    ParseContext ctx() const {
        ParseContext c{};
        c.senderGuidPrefix = writer.prefix;
        return c;
    }
};

template <uint8_t Slots>
class Simulation {
public:
    Simulation(const SimConfig& cfg, uint64_t seed) : cfg_(cfg), rng_{seed} {
        (void)proxies_.reserve(cfg.readers);
        for (uint8_t i = 0; i < cfg.readers; ++i) {
            SimWriter& w = writers_[i];
            w.reader.prefix.value[0] = 0x51u;
            w.reader.entityId = make_entity(1u, i, 1u, 0x04u);
            w.writer.prefix.value[0] = static_cast<Octet>(0x60u + i);
            w.writer.entityId = make_entity(2u, i, 2u, 0x03u);
            w.failure = &failure_;
            w.r = std::make_unique<SharedPoolStatefulReader>(
                w.reader, ReliabilityKind::RELIABLE, tx_, proxies_, 1u, 1u, assembly_, 1u, held_,
                cfg.limit, SimWriter::on_change, &w);
            (void)w.r->matched_writer_add(w.writer, &loc_, 1u);
        }

        // Take every block a sample may use (a full class promotes to the
        // next; the largest is kept for large samples), then give back
        // `spare_blocks` of the class samples land in, which is also the
        // class an ACKNACK is built in. Held changes then compete for blocks
        // with the change that closes a gap and with the reader's ACKNACKs.
        if (cfg.spare_blocks > 0u) {
            for (BufRef b = buf_alloc(kSampleSize); b.valid(); b = buf_alloc(kSampleSize)) {
                hog_.push_back(static_cast<BufRef&&>(b));
            }

            hog_.erase(hog_.begin(), hog_.begin() + cfg.spare_blocks);
        }

        tx_.sent.clear();
        silence_step_ = cfg.silent ? rng_.below(cfg.lossy_steps + 1u) : UINT32_MAX;
    }

    /// Runs the lossy phase then the fair phase; empty on success, else the first broken property.
    std::string run() {
        const uint32_t max_steps = cfg_.lossy_steps + 400000u;
        for (uint32_t step = 0; step < max_steps && failure_.empty(); ++step) {
            if (step == silence_step_) {
                writers_[0].alive = false;
            }

            const bool lossy = step < cfg_.lossy_steps;
            if (!lossy && settled()) {
                break;
            }

            g_stateful_reader_test_now_ms += 1u + rng_.below(20u);
            act(lossy);
            for (uint8_t i = 0; i < cfg_.readers; ++i) {
                writers_[i].r->tick(g_stateful_reader_test_now_ms);
            }

            drain_acknacks(lossy);
            if (held_.count() > Slots) {
                failure_ = "WithinCap: the pool holds more than its slots";
            }
        }

        if (failure_.empty()) {
            check_every_change_done();
        }

        if (!failure_.empty()) {
            failure_ += describe();
        }

        return failure_;
    }

private:
    /// One step of a writer or of the network.
    void act(bool lossy) {
        const uint32_t roll = rng_.below(100u);
        SimWriter&     w = writers_[rng_.below(cfg_.readers)];
        const uint8_t  to = static_cast<uint8_t>(&w - writers_);
        if (roll < 15u) {
            if (w.alive && w.written < cfg_.changes) {
                ++w.written;
                net_.push_back({false, to, w.written, 0});
            }

            return;
        }

        if (roll < 25u) {
            if (w.alive && w.acked < w.written) {
                net_.push_back({true, to, w.acked + 1, w.written});
            }

            return;
        }

        if (net_.empty()) {
            return;
        }

        const size_t pick = rng_.below(static_cast<uint32_t>(net_.size()));
        const Msg    m = net_[pick];
        const bool   duplicate = rng_.percent(5u);
        if (!duplicate) {
            net_[pick] = net_.back();
            net_.pop_back();
        }

        if (!(lossy && rng_.percent(cfg_.loss_pct))) {
            receive(m);
        }
    }

    void receive(const Msg& m) {
        SimWriter& w = writers_[m.to];
        if (m.heartbeat) {
            ParsedHeartbeat hb{};
            hb.readerId = w.reader.entityId;
            hb.writerId = w.writer.entityId;
            hb.firstSN = SequenceNumber::from_int64(m.sn);
            hb.lastSN = SequenceNumber::from_int64(m.last);
            hb.count = ++w.hb_count;
            w.r->on_heartbeat(w.ctx(), hb);
            return;
        }

        static const Octet payload[kSampleSize] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        ParsedData         d{};
        d.readerId = w.reader.entityId;
        d.writerId = w.writer.entityId;
        d.writerSN = SequenceNumber::from_int64(m.sn);
        d.dataFlag = true;
        d.serializedPayload = payload;
        d.serializedPayloadSize = sizeof(payload);
        w.r->on_data(w.ctx(), d);
    }

    /// Hands each ACKNACK the readers sent to its writer, which resends what it requests.
    void drain_acknacks(bool lossy) {
        std::vector<std::vector<Octet>> sent;
        sent.swap(tx_.sent);
        for (const std::vector<Octet>& msg : sent) {
            if (lossy && rng_.percent(cfg_.loss_pct)) {
                continue;
            }

            ParsedAckNack ack{};
            ParseOps      ops{};
            ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
                *static_cast<ParsedAckNack*>(c.user_data) = a;
            };
            if (parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                              &ack)
                != ParseResult::OK) {
                continue;
            }

            for (uint8_t i = 0; i < cfg_.readers; ++i) {
                if (writers_[i].writer.entityId == ack.writerId) {
                    answer(i, ack);
                }
            }
        }
    }

    /// The writer's side of §8.4.9.2.8: acknowledge below the base, resend the requested set.
    void answer(uint8_t i, const ParsedAckNack& ack) {
        SimWriter&    w = writers_[i];
        const int64_t base = ack.readerSNState.bitmapBase.to_int64();
        if (base - 1 > w.delivered && failure_.empty()) {
            failure_ = "NoAckPastUndelivered: reader " + std::to_string(i) + " acknowledged "
                       + std::to_string(base - 1) + " having delivered "
                       + std::to_string(w.delivered);
        }

        if (!w.alive) {
            return;
        }

        if (base - 1 > w.acked) {
            w.acked = base - 1;
        }

        for (uint32_t b = 0; b < ack.readerSNState.numBits; ++b) {
            const int64_t sn = base + b;
            if (ack.readerSNState.test(b) && sn <= w.written) {
                net_.push_back({false, i, sn, 0});
            }
        }
    }

    /// True once every live writer has written and had acknowledged every change.
    bool settled() const {
        for (uint8_t i = 0; i < cfg_.readers; ++i) {
            const SimWriter& w = writers_[i];
            if (w.alive && (w.written < cfg_.changes || w.acked < cfg_.changes)) {
                return false;
            }
        }

        return true;
    }

    /// The end state, appended to a failure so it can be read without a debugger.
    std::string describe() const {
        uint32_t free = 0;
        for (uint8_t c = 0; c < g_buf_pool.class_count(); ++c) {
            free += g_buf_pool.class_info(c).available;
        }

        std::string out = " [free blocks " + std::to_string(free) + ", held "
                          + std::to_string(held_.count()) + ", in flight "
                          + std::to_string(net_.size());
        for (uint8_t i = 0; i < cfg_.readers; ++i) {
            const SimWriter& w = writers_[i];
            out += "; w" + std::to_string(i) + (w.alive ? "" : " silent") + " written "
                   + std::to_string(w.written) + " acked " + std::to_string(w.acked) + " delivered "
                   + std::to_string(w.delivered);
        }

        return out + "]";
    }

    void check_every_change_done() {
        for (uint8_t i = 0; i < cfg_.readers; ++i) {
            const SimWriter& w = writers_[i];
            if (w.alive && w.delivered != cfg_.changes) {
                failure_ = "EveryChangeDone: reader " + std::to_string(i) + " delivered "
                           + std::to_string(w.delivered) + " of " + std::to_string(cfg_.changes);
                return;
            }
        }
    }

    SimConfig                        cfg_;
    Rng                              rng_;
    CaptureTransport                 tx_{};
    ProxyPoolStorage<WriterProxy, 4> proxies_;
    FragmentAssemblyPoolStorage<1>   assembly_;
    HeldChangePoolStorage<Slots>     held_;
    const Locator                    loc_ = make_udpv4_locator(0xC0A801B0u, 7411u);
    SimWriter                        writers_[4];
    std::vector<Msg>                 net_{};
    std::vector<BufRef>              hog_{};
    uint32_t                         silence_step_ = UINT32_MAX;
    std::string                      failure_{};
};

/// Free blocks over every class of the buffer pool.
uint32_t free_blocks() {
    uint32_t n = 0;
    for (uint8_t c = 0; c < g_buf_pool.class_count(); ++c) {
        n += g_buf_pool.class_info(c).available;
    }

    return n;
}

uint32_t env_u32(const char* name, uint32_t fallback) {
    const char* v = std::getenv(name);
    return v ? static_cast<uint32_t>(std::strtoul(v, nullptr, 10)) : fallback;
}

/// Runs `cfg` for every seed (or the one in ROSETTE_SIM_SEED); false with the first failure.
template <uint8_t Slots>
bool run_seeds(const SimConfig& cfg, uint64_t salt, std::string& err) {
    const uint32_t free_before = free_blocks();
    const char*    one = std::getenv("ROSETTE_SIM_SEED");
    const uint64_t first = one ? std::strtoull(one, nullptr, 10) : 1u;
    const uint64_t count = one ? 1u : env_u32("ROSETTE_SIM_SEEDS", 200u);
    for (uint64_t seed = first; seed < first + count; ++seed) {
        std::string failure;
        {
            Simulation<Slots> sim(cfg, seed ^ salt);
            failure = sim.run();
        }

        if (failure.empty() && free_blocks() != free_before) {
            failure = "a buffer block was not released";
        }

        if (!failure.empty()) {
            err = "seed " + std::to_string(seed) + ": " + failure;
            return false;
        }
    }

    return true;
}

}  // namespace

// §8.4.10.4: readers sharing a hold pool, each with its own writer on a lossy
// network, deliver every change once, in order, and acknowledge nothing
// undelivered.
static bool tc_srr_8_4_10_4_020(std::string& err) {
    return run_seeds<8>({2u, 4u, 40, 25u, 3000u, 0u, false}, 0x2000u, err)
           && run_seeds<8>({4u, 4u, 40, 25u, 4000u, 0u, false}, 0x2001u, err)
           && run_seeds<2>({2u, 2u, 40, 25u, 3000u, 0u, false}, 0x2002u, err);
}

// §8.4.10.4: a writer that stops with changes held does not keep the other
// readers from finishing.
static bool tc_srr_8_4_10_4_021(std::string& err) {
    return run_seeds<8>({3u, 4u, 40, 25u, 3000u, 0u, true}, 0x2100u, err)
           && run_seeds<2>({2u, 2u, 40, 25u, 3000u, 0u, true}, 0x2101u, err);
}

// §8.4.10.4: with few buffer blocks to spare, held changes give theirs up to
// the changes that close gaps, and every live reader still finishes.
static bool tc_srr_8_4_10_4_022(std::string& err) {
    return run_seeds<8>({2u, 4u, 40, 25u, 3000u, 3u, false}, 0x2200u, err)
           && run_seeds<8>({3u, 4u, 40, 25u, 3000u, 3u, true}, 0x2201u, err)
           && run_seeds<2>({2u, 2u, 40, 25u, 3000u, 2u, true}, 0x2202u, err);
}

void append_stateful_reader_tests_8_4_10_4_simulated_writers(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.10.4-020",
                     "§8.4.10.4: readers sharing a hold pool deliver every change once, in "
                     "order, over a lossy network, and acknowledge nothing undelivered.",
                     tc_srr_8_4_10_4_020});
    tests.push_back({"TC-SRR-8.4.10.4-021",
                     "§8.4.10.4: a writer that stops with changes held does not keep the other "
                     "readers from finishing.",
                     tc_srr_8_4_10_4_021});
    tests.push_back({"TC-SRR-8.4.10.4-022",
                     "§8.4.10.4: with buffer blocks scarce, every live reader still finishes.",
                     tc_srr_8_4_10_4_022});
}

}  // namespace spec_test
