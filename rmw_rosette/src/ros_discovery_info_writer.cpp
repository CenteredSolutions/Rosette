/**
 * @file ros_discovery_info_writer.cpp
 * @brief RosDiscoveryInfoWriter: publishes ParticipantEntitiesInfo on
 *        `ros_discovery_info` (ros_discovery_info_writer.hpp).
 */

#include "rmw_rosette/ros_discovery_info_writer.hpp"

#include "rosette/rtps/buf_pool.hpp"

namespace rmw_rosette {

bool RosDiscoveryInfoWriter::create(rtps::RTPSParticipant& participant) noexcept {
    rtps::WriterConfig cfg{};
    cfg.topicName.assign(kRosDiscoveryInfoTopic);
    cfg.typeName.assign(kParticipantEntitiesInfoType);
    cfg.reliability = rtps::ReliabilityKind::RELIABLE;
    cfg.durability = rtps::DurabilityKind::TRANSIENT_LOCAL;
    writer_ = participant.create_writer<1>(cfg);
    if (writer_ == nullptr) {
        participant_ = nullptr;
        return false;
    }

    participant_ = &participant;
    // Every ROS 2 participant reads this writer, so its matches keep no peer
    // in the participant's active tier; parked peers are reactivated when
    // its contents change instead (publish_change()).
    participant.set_discovery_writer(writer_->guid());
    writer_->set_rematch_listener(&RosDiscoveryInfoWriter::on_reader_rematched, this);
    return true;
}

void RosDiscoveryInfoWriter::on_reader_rematched(void* self) noexcept {
    (void)static_cast<RosDiscoveryInfoWriter*>(self)->publish();
}

bool RosDiscoveryInfoWriter::add_node(const Node& node) noexcept {
    if (!entities_.add_node(node)) {
        return false;
    }

    if (publish_change()) {
        return true;
    }

    entities_.remove_node(node);
    return false;
}

void RosDiscoveryInfoWriter::remove_node(const Node& node) noexcept {
    entities_.remove_node(node);
    (void)publish_change();
}

bool RosDiscoveryInfoWriter::add_endpoints(const Node& node, const rtps::Guid* reader,
                                           const rtps::Guid* writer) noexcept {
    bool ok = true;
    if (reader) {
        ok = entities_.associate(node, *reader, true);
    }

    if (ok && writer) {
        ok = entities_.associate(node, *writer, false);
    }

    if (ok && publish_change()) {
        return true;
    }

    if (reader) {
        entities_.dissociate(*reader);
    }

    if (writer) {
        entities_.dissociate(*writer);
    }

    return false;
}

void RosDiscoveryInfoWriter::remove_endpoints(const rtps::Guid* reader,
                                              const rtps::Guid* writer) noexcept {
    if (reader) entities_.dissociate(*reader);
    if (writer) entities_.dissociate(*writer);
    (void)publish_change();
}

bool RosDiscoveryInfoWriter::publish_change() noexcept {
    if (!publish()) {
        return false;
    }

    participant_->discovery_info_changed();
    return true;
}

bool RosDiscoveryInfoWriter::publish() noexcept {
    if (!writer_) return false;
    uint32_t request = RTPS_POOL_CLASS0_SIZE;
    for (;;) {
        rtps::BufRef buf = rtps::buf_alloc(request);
        if (!buf.valid()) return false;
        const uint32_t len =
            entities_.encode(participant_->participant_guid(), buf.data, buf.capacity());
        if (len > 0u) {
            buf.size = len;
            return writer_->write_async_owned(rtps::ChangeKind::ALIVE,
                                              static_cast<rtps::BufRef&&>(buf));
        }
        if (buf.capacity() >= rtps::kPoolMaxBlockSize) return false;
        request = buf.capacity() + 1u;
    }
}

}  // namespace rmw_rosette
