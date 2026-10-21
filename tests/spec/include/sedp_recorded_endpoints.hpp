/**
 * @file sedp_recorded_endpoints.hpp
 * @brief A LocalEndpointSource for SEDP tests, and a SedpWriter that uses it in rebuild builds.
 *
 * A rebuilding SedpWriter reads each local endpoint back from its
 * LocalEndpointSource for every send; in a participant that is the endpoint
 * table. Tests that drive a SedpWriter on its own record the data they
 * register here instead. sedp_spec_tests is built twice, once with
 * SPEC_SEDP_REBUILD_ANNOUNCEMENTS=1, so every SEDP writer test that
 * constructs a SpecSedpWriter runs against both announcement storage modes.
 */

#pragma once

#include "rosette/rtps/sedp.hpp"

#include <vector>

#ifndef SPEC_SEDP_REBUILD_ANNOUNCEMENTS
#define SPEC_SEDP_REBUILD_ANNOUNCEMENTS 0
#endif

namespace spec_test {

using namespace rosette::rtps;

/**
 * @brief Local endpoint data a test registers, served back as a LocalEndpointSource.
 *
 * Lookups go by GUID, so an endpoint a test erases makes the source report
 * it unknown, as a participant does once the endpoint is deleted.
 */
struct RecordedEndpoints {
    std::vector<DiscoveredWriterData> writers{};
    std::vector<DiscoveredReaderData> readers{};

    /// @brief The source a rebuilding SedpWriter reads these endpoints from.
    LocalEndpointSource source() noexcept {
        return LocalEndpointSource{describe_writer, describe_reader, this};
    }

    /// @brief LocalEndpointSource::DescribeWriter over `writers`.
    static bool describe_writer(void* user, const Guid& guid, DiscoveredWriterData& out) noexcept {
        for (const DiscoveredWriterData& wd : static_cast<RecordedEndpoints*>(user)->writers) {
            if (wd.writerGuid == guid) {
                out = wd;
                return true;
            }
        }

        return false;
    }

    /// @brief LocalEndpointSource::DescribeReader over `readers`.
    static bool describe_reader(void* user, const Guid& guid, DiscoveredReaderData& out) noexcept {
        for (const DiscoveredReaderData& rd : static_cast<RecordedEndpoints*>(user)->readers) {
            if (rd.readerGuid == guid) {
                out = rd;
                return true;
            }
        }

        return false;
    }
};

/**
 * @brief SedpWriter that records what it registers, rebuilding announcements
 *   when the test binary is built with SPEC_SEDP_REBUILD_ANNOUNCEMENTS=1.
 *
 * Registration records the endpoint before announcing it, the order the
 * participant keeps between its endpoint table and SEDP.
 */
class SpecSedpWriter : private RecordedEndpoints, public SedpWriter {
public:
    SpecSedpWriter(const GuidPrefix& local_prefix, ITransport& transport) noexcept
        : RecordedEndpoints(),
          SedpWriter(local_prefix, transport,
                     SPEC_SEDP_REBUILD_ANNOUNCEMENTS != 0 ? source() : LocalEndpointSource{}) {}

    /// @brief Records `wd`, then SedpWriter::register_writer().
    bool register_writer(const DiscoveredWriterData& wd) noexcept {
        writers.push_back(wd);
        return SedpWriter::register_writer(wd);
    }

    /// @brief Records `rd`, then SedpWriter::register_reader().
    bool register_reader(const DiscoveredReaderData& rd) noexcept {
        readers.push_back(rd);
        return SedpWriter::register_reader(rd);
    }
};

}  // namespace spec_test
