# Stark Motor Control API

## Files needed

Copy these two files to your project:
- `stark_client.h` -- all APIs, header-only, zero dependency
- `stark_shm.h`   -- SHM data structures

## Compile

```bash
gcc -O2 your_algo.c -lpthread -lrt -lm -o your_algo
```

## Minimal template

```c
#include "stark_client.h"

int main() {
    stark_client_t c;
    stark_open(&c);

    while (!stark_ready(&c)) usleep(100000);  // wait for calibration

    stark_enable(&c, 1); stark_enable(&c, 2);
    usleep(5000);                               // let RT thread process
    stark_set_mode(&c, 1, 5); stark_set_mode(&c, 2, 5);  // current mode
    usleep(5000);

    while (1) {
        motor_data_t m1 = stark_fb(&c, 1);
        motor_data_t m2 = stark_fb(&c, 2);
        imu_data_t   imu = stark_imu(&c);

        int32_t t1 = your_control(m1, m2, imu);  // your algorithm
        int32_t t2 = your_control(m2, m1, imu);

        stark_multi(&c, t1, 0, 0, t2, 0, 0);    // send torque to both motors
        usleep(1000);                             // 1KHz
    }

    stark_estop(&c, 1); stark_estop(&c, 2);
    stark_close(&c);
}
```

## API reference

### Lifecycle

| function | description |
|----------|-------------|
| `stark_open(&c)` | connect SHM, returns 0 on success |
| `stark_close(&c)` | disconnect |

### Status

| function | returns |
|----------|---------|
| `stark_ready(&c)` | 1 = calib done, motors ready |
| `stark_online(&c, id)` | 1 = motor online |
| `stark_state(&c)` | 0=BOOTING 1=READY 2=RUNNING 3=FAULT |

### Feedback (zero-copy, from SHM)

```c
motor_data_t stark_fb(&c, id);           // id=1(right) 2(left)
// fields: position(counts), velocity(RPM), current_iq(mA), temperature(0.1C),
//         status_byte, error_code(uint16)

imu_data_t stark_imu(&c);
// fields: roll, pitch, yaw(deg), acc_x/y/z(g), gyro_x/y/z(dps), quat_w/x/y/z
```

### Periodic upload (5ms, auto-start after calibration)

```c
const PeriodicUploadData* d = stark_report_data(&c);  // NULL if not enabled
uint32_t ver = stark_report_version(&c);              // monotonic, detect update
// PeriodicUploadData fields:
//   motor: RealtimeVelocity(RPM*10), motor_abs_angle(deg*10),
//          cal_Iq_current(mA), motor_temp(C*100)
//   sensor: hall_a/b/c, df181_torque, knee_angle, key_landing
//   imu:   gyro_dps_x/y/z, quat_w/x/y/z, gyro_roll/pitch/yaw, acc_x/y/z
//   sync:  frame_cycle, motor_ts_us, imu_ts_us, sensor_ts_us
```

### Control commands (real-time, 1KHz PDO)

| function | args | mode needed |
|----------|------|-------------|
| `stark_torque(c, id, mA)` | id=1/2, mA=current | 5 (current) |
| `stark_multi(c, t1,v1,p1, t2,v2,p2)` | both motors in one frame | 5 |
| `stark_speed(c, id, rpm)` | RPM | 4 (CSV) |
| `stark_position(c, id, deg)` | absolute deg | 3 (CSP) |
| `stark_pp(c, id, deg, accel, vel)` | deg, RPM/s, RPM | 1 (PP) |
| `stark_pv(c, id, rpm, accel)` | RPM, RPM/s | 2 (PV) |
| `stark_mit(c, id, pos,vel, kp,kd, tor)` | impedance | 6 (MIT) |

`stark_multi` is recommended for dual motor control -- one CANFD frame for both.

### Management commands (non-RT, must be separated from control by 5ms)

| function | effect |
|----------|--------|
| `stark_enable(c, id)` | enable motor |
| `stark_disable(c, id)` | disable motor |
| `stark_estop(c, id)` | emergency stop (disable + bus off) |
| `stark_set_mode(c, id, mode)` | set control mode |
| `stark_clear_fault(c, id)` | clear fault |

### Mode table

| mode | name | description |
|------|------|-------------|
| 1 | PP | profile position, drive-side ramp |
| 2 | PV | profile velocity, drive-side ramp |
| 3 | CSP | cyclic sync position, SYNC-triggered |
| 4 | CSV | cyclic sync velocity, SYNC-triggered |
| 5 | Current | Q-axis current direct control |
| 6 | MIT | impedance control (pos+stiffness+damping) |

## Key rules

1. Management commands (enable/disable/estop/set_mode) and control commands (torque/multi/etc.) must be separated by at least 5ms. Send management first, sleep, then enter control loop.

2. Use `stark_multi` for dual motor control. One CANFD frame, better sync.

3. Send a control command every cycle. Even if target unchanged, keep sending. If idle for 500ms, safety monitor will fault-stop the motors.

4. Motor IDs: 1=right hip, 2=left hip. Motor count and IDs come from CAN hardware, not configurable.

5. Run with `sudo` -- SHM needs root if stark_node created it as root.
