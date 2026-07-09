# Stark Motor Control API

## Files

- `stark_client.h` -- All APIs, header-only, zero dependencies
- `stark_shm.h`   -- Shared memory data structures

## Build

```bash
gcc -O2 your_algo.c -lpthread -lrt -lm -o your_algo
g++ -O2 your_algo.cpp -lpthread -lrt -lm -o your_algo
```

## Template

```c
#include "stark_client.h"
int main() {
    stark_client_t c;

    while (stark_open(&c) != 0) usleep(100000);
    while (!stark_ready(&c)) usleep(100000);

    stark_enable(&c, 1);  stark_enable(&c, 2);
    stark_set_mode(&c, 1, STARK_MODE_CURRENT);
    stark_set_mode(&c, 2, STARK_MODE_CURRENT);
    usleep(5000);

    uint32_t rpt_ver = 0, rt_cycle = 0;
    while (1) {
        motor_data_t m1 = stark_fb(&c, 1);
        motor_data_t m2 = stark_fb(&c, 2);
        imu_data_t  imu = stark_imu(&c);

        stark_multi(&c, t1, 0, 0, t2, 0, 0);
        usleep(1000);

        const PeriodicUploadData* d;
        if (stark_report_try_read(&c, &rpt_ver, &d)) { }

        static int tick = 0;
        if (++tick % 200 == 0) {
            stark_heartbeat(&c);
            if (!stark_rt_alive(&c, &rt_cycle)) {
                while (!stark_ready(&c)) usleep(100000);
                stark_enable(&c, 1); stark_enable(&c, 2);
                usleep(5000);
            }
        }
    }
    stark_estop(&c, 1); stark_estop(&c, 2);
    stark_close(&c);
}
```

## API Reference

### Lifecycle

| Function | Returns | Description |
|----------|---------|-------------|
| `stark_open(c)` | 0=ok, -1=fail | Connect SHM. Retry loop to wait for stark_node startup |
| `stark_close(c)` | void | Disconnect SHM |

### Status

| Function | Returns |
|----------|---------|
| `stark_ready(c)` | 1=calibration done, ready to control |
| `stark_online(c, id)` | 1=motor online (id: 1=right, 2=left) |
| `stark_state(c)` | 0=BOOTING, 1=READY, 2=RUNNING, 3=FAULT |
| `stark_calib(c)` | 0=idle, 1=calibrating, 2=done, 3=timeout |
| `stark_severity(c)` | 0=OK, 1=WARN, 2=FAULT |
| `stark_fault_reason(c)` | Fault code (fault_reason_t) |

### Feedback (zero-copy, reads SHM double buffer)

```c
motor_data_t stark_fb(stark_client_t* c, int id);
//   position     encoder counts [-32768,32767], deg = pos * 360 / 65536
//   velocity     RPM
//   current_iq   mA
//   temperature  0.1C (divide by 10 for degC)
//   status_byte  bit7=enable, bit6=brake, bit5=fault, bit4=in-position
//   mode         current control mode
//   error_code   fault code

imu_data_t stark_imu(stark_client_t* c);
//   roll, pitch, yaw     Euler angles (deg)
//   acc_x, acc_y, acc_z  Acceleration (g)
//   gyro_x, gyro_y, gyro_z  Angular velocity (dps)
//   quat_w/x/y/z         9-axis quaternion
//   mag_x, mag_y, mag_z  Magnetometer (uT)
//   heading_deg          Heading angle
//   stationary           0=moving, 2=stationary
//   gyr_accuracy         0=uncal, 3=precision

stark_sensor_data_t stark_sensor(stark_client_t* c, int id);
//   hall_adc0/1/2  Hall sensor ADC (0-4095)
//   force_raw      DF181 torque raw (0-16383)
//   knee_hall      Knee hall sensor (0-4095)
//   key_landing    Foot contact switch
//   data_valid     Torque data valid

barometer_data_t stark_baro(stark_client_t* c);
//   pressure_hpa, temperature_c, altitude_m
```

### Control Commands (realtime, 1kHz PDO)

| Function | Params | Mode |
|----------|--------|------|
| `stark_multi(c, t1,v1,p1, t2,v2,p2)` | t=mA, v=RPM, p=deg | Current mode |
| `stark_torque(c, id, mA)` | current mA | CURRENT |
| `stark_speed(c, id, rpm)` | target RPM | CSV |
| `stark_position(c, id, deg)` | absolute angle | CSP |
| `stark_pp(c, id, deg, acc, vel)` | profile pos (deg), accel (RPM/s), vel (RPM) | PP |
| `stark_pv(c, id, rpm, acc)` | profile vel (RPM, RPM/s) | PV |
| `stark_mit(c, id, pos,vel, kp,kd, tor)` | MIT impedance | MIT |

Recommended: use `stark_multi` for dual-motor sync over one CANFD frame.

`stark_multi` params follow the current mode:
- CURRENT: t1/t2 only (current), v/p must be 0
- CSP: p1/p2 = absolute angle
- CSV/PV: v1/v2 = speed

### SDO Control (single-frame, via mailbox)

| Function | Params |
|----------|--------|
| `stark_sdo_cur(c, id, mA)` | SDO current |
| `stark_sdo_pos(c, id, deg, acc, vel)` | SDO profile position |
| `stark_sdo_vel(c, id, rpm, acc)` | SDO profile velocity |

### Management Commands (independent channel, 5ms gap to control)

| Function | Description |
|----------|-------------|
| `stark_enable(c, id)` | Enable motor |
| `stark_disable(c, id)` | Disable + release brake + current=0 |
| `stark_estop(c, id)` | Emergency stop: disable + brake lock + current=0 |
| `stark_recover(c, id)` | Recover from estop |
| `stark_clear_fault(c, id)` | Clear fault + auto-enable + current mode |
| `stark_set_mode(c, id, mode)` | Switch control mode |

### Heartbeat

| Function | Description |
|----------|-------------|
| `stark_heartbeat(c)` | Declare algorithm alive, call every 200ms |
| `stark_rt_alive(c, &last)` | Check stark_node reverse liveness, 0=reconnect needed |

Default timeout 1000ms (configurable). On timeout, node auto-disables both motors without FAULT. Re-enable via `stark_enable` after heartbeat resumes.

### Periodic Report (auto-enabled after calibration, 5ms push)

| Function | Returns | Description |
|----------|---------|-------------|
| `stark_report_data(c)` | PeriodicUploadData* | Data pointer, NULL if not enabled |
| `stark_report_version(c)` | uint32_t | Monotonic version, compare for updates |

```c
/* Polling: for main control loop */
uint32_t ver = 0;
const PeriodicUploadData* d;
if (stark_report_try_read(&c, &ver, &d)) {
    // d->RealtimeVelocity (RPM*10), d->motor_abs_angle (deg*10)
    // d->cal_Iq_current (A*100), d->motor_temp (C*100)
    // d->hall_a_data/b_data/c_data, d->df181_torque, d->knee_hall, d->key_landing
    // d->spi_torque/_left (float, torque = raw counts / 100)
    // d->spi_valid/_left (uint8, 1=valid), d->spi_error/_left (uint8, device error code)
    // IMU: d->gyro_roll/pitch/yaw, d->quat_w/x/y/z, d->acc_x/y/z
}

/* Blocking wait: for dedicated receive thread */
if (stark_report_wait(&c, &ver, &d, 100) == 1) {
    // new data available, d/ver valid
}
```

### Utilities

| Function | Description |
|----------|-------------|
| `stark_request_calib(c)` | Request complex calibration |

## Control Modes

| Constant | Value | Description |
|----------|-------|-------------|
| STARK_MODE_PP | 1 | Profile position, trapezoidal ramp on drive board |
| STARK_MODE_PV | 2 | Profile velocity, trapezoidal ramp on drive board |
| STARK_MODE_CSP | 3 | Cyclic sync position, SYNC triggered |
| STARK_MODE_CSV | 4 | Cyclic sync velocity, SYNC triggered |
| STARK_MODE_CURRENT | 5 | Q-axis current direct control (recommended for exoskeleton) |
| STARK_MODE_MIT | 6 | MIT impedance control |

## Motor IDs

- 1: Right hip
- 2: Left hip

## Rules

1. Run as root (SHM created by stark_periph_manager_node as root)
2. Management commands require 5ms gap before control commands
3. Use `stark_multi` for dual-motor sync
4. Call `stark_heartbeat` every 200ms; timeout auto-disables motors
5. Exoskeleton: no command sent = feedback still updates, joint free, no disable
6. `stark_open` can retry loop to wait for stark_node startup

## Configuration (config.json)

```json
{
  "motors": [
    { "id": 1, "auto_enable": false },
    { "id": 2, "auto_enable": false }
  ],
  "sensor": {
    "period_div": 1,
    "bus_format": 3,
    "mode": 2,
    "force_module": 1
  },
  "safety": {
    "heartbeat_timeout_ms": 1000,
    "overtemp_celsius": 80,
    "can_offline_ms": 2000,
    "encoder_stall_s": 3
  }
}
```

### sensor

| Field | Values | Description |
|-------|--------|-------------|
| period_div | 1..65535 | Report period divider, 0.5ms base. 1 = 2000Hz |
| bus_format | 0 / 3 | 0 = Classic CAN, 3 = CANFD BRS |
| mode | 0 / 1 / 2 | 0 = off, 1 = 0x680 only, 2 = 0x680 + 0x6B0 |
| force_module | 0 / 1 | 0 = torque via 0x680, 1 = torque via 0x6B0 SPI frame |

## demo_algo Usage

```bash
# Control modes
./demo_algo torque <mA>          # current sine, +/-mA
./demo_algo speed <rpm>          # CSV velocity trapezoid
./demo_algo pos <deg>            # CSP position
./demo_algo pp <deg> [acc] [vel] # Profile position
./demo_algo pv <rpm> [acc]       # Profile velocity
./demo_algo mit <kp> <kd>        # MIT impedance
./demo_algo multi <ma1> <ma2>    # Multi-axis broadcast

# SDO single-frame
./demo_algo sdo cur <id> <mA>
./demo_algo sdo pos <id> <deg>
./demo_algo sdo vel <id> <rpm>

# Management
./demo_algo enable/disable/estop/clearf <id>

# Status
./demo_algo stat                 # feedback read-only
./demo_algo report               # periodic report loop
```
