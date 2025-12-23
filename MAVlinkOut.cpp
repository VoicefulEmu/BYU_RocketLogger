#include "MavlinkOut.h"
#include <MAVLink.h>

static const uint8_t SYS_ID  = 1;
static const uint8_t COMP_ID = MAV_COMP_ID_IMU;

void mavlink_setup() {
  // nothing yet; kept for symmetry if you later add heartbeat timers, etc.
}

void mavlink_send_sample(const LogSample &s) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // time_usec in HIGHRES_IMU is uint64_t, cast t_us
  uint64_t t_us_64 = (uint64_t)s.t_us;

  mavlink_msg_highres_imu_pack(
      SYS_ID, COMP_ID, &msg,
      t_us_64,              // time_usec
      s.ax, s.ay, s.az,     // xacc, yacc, zacc
      s.gx, s.gy, s.gz,     // xgyro, ygyro, zgyro
      s.mx, s.my, s.mz,     // xmag, ymag, zmag
      s.pressPa,            // abs_pressure
      0.0f,                 // diff_pressure
      s.altM,               // pressure_alt
      s.imuTempC,           // temperature
      0,                    // fields_updated (uint16_t)
      0                     // id (uint8_t) – sensor ID
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}
