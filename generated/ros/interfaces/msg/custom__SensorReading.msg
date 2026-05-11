# Custom message: all primitive types + arrays + bounded string + constants
uint8 MAX_SENSORS=8
float32 INVALID_VALUE=-1.0

std_msgs/Header header
uint8 sensor_id
bool active
int8 status
int16 raw_value
int32 sequence
int64 timestamp_ns
uint8 flags
uint16 checksum
uint32 error_code
uint64 uptime_us
float32 temperature
float64 voltage
string<=32 label
float32[3] accel
float64[] samples
uint8[<=16] payload
