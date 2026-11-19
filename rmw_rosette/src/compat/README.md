# Vendor compatibility

rmw_rosette speaks the OMG wire contracts (DDSI-RTPS, DDS-RPC) by default. A stock ROS 2 middleware that departs
from them gets its dialect here, one module per departure, selected by the peer's SPDP vendor ID
(`RTPSParticipant::peer_vendor_id()`). Headers live in `include/rmw_rosette/compat/`.

| Module                       | Vendor (ID)                  | Departure                                          |
|------------------------------|------------------------------|----------------------------------------------------|
| `cyclone_request_header.cpp` | Eclipse Cyclone DDS (0x0110) | Service request identity in a 16-octet CDR header, |
|                              |                              | not PID_RELATED_SAMPLE_IDENTITY                    |
| `cyclone_service_user_data.cpp` | Eclipse Cyclone DDS (0x0110) | Service and client endpoints paired by SEDP     |
|                              |                              | USER_DATA `serviceid=` / `clientid=`; announced to |
|                              |                              | every peer, ignored by the others                  |
