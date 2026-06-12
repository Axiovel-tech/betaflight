/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor_impl.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"
#include "drivers/pwm_output.h"
#include "drivers/servo_impl.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "timer_def.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "config/config_eeprom_impl.h"

#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/gps.h"
#include "io/gps_virtual.h"

#include "dyad.h"
#include "udplink.h"

uint32_t SystemCoreClock;

static fdm_packet fdmPkt;
static rc_packet rcPkt;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt;

static bool rc_received = false;
static bool fdm_received = false;

static struct timespec start_time;
static double simRate = 1.0;
static pthread_t tcpWorker, udpWorker, udpWorkerRC;
static bool workerRunning = true;
static udpLink_t stateLink, pwmLink, pwmRawLink, rcLink;
static pthread_mutex_t updateLock;
static pthread_mutex_t mainLoopLock;
static char simulator_ip[32] = "127.0.0.1";

#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In

// AV fork: snapshot of the last micros64() result for cross-thread readers.
// micros64() itself is NOT thread-safe in stock mode (static accumulator
// advanced by wall-delta * simRate); it must only ever be called from the
// main loop. The TCP serial RX callback path (CRSF bytes, dyad thread)
// needs a timestamp, so it reads this monotonic snapshot instead -- at the
// main loop's ~9 kHz call rate it is at most ~0.1 ms stale, far below the
// CRSF inter-frame gap the consumer measures.
// (Hoisted above updateState() for AXIO-LOCKSTEP, which updates it from
// the FDM thread on every clock step.)
static _Atomic uint64_t microsSnapshot = 0;

// ---- AXIO-LOCKSTEP (AV fork, sim-only) ---------------------------------
// Deterministic lockstep mode, enabled with SITL_LOCKSTEP=1 in the
// environment (default off: stock free-running behaviour, bit-identical
// codepaths). In lockstep mode:
//
//  * micros64()/millis64() are derived exclusively from FDM packet
//    timestamps (Gazebo sim time); the wall clock never advances them.
//    Between FDM packets the firmware clock is FROZEN, so pausing Gazebo
//    pauses Betaflight, and Gazebo running faster/slower than real time
//    is invisible to the firmware.
//  * Before the first streamed FDM packet ("engagement") the virtual
//    clock advances only when the firmware itself sleeps (delay()/
//    delayMicroseconds()), so boot consumes a deterministic amount of
//    virtual time regardless of host load.
//  * The PID/main loop is gated 1:1 on FDM arrival via
//    SIMULATOR_GYROPID_SYNC + lockMainPID() (one PID iteration, hence
//    exactly one motor packet, per FDM packet).
//  * Optional: SITL_LOCKSTEP_RC_PLAN=<file> loads a scripted RC plan
//    (lines: "<t_seconds> <ch0> <ch1> ... <chN>", up to 16 channels,
//    missing channels default to 1500, '#' comments). Rows are applied
//    synchronously in updateState() keyed to the VIRTUAL clock, before
//    the clock advances past them -- a fully deterministic RC source
//    (no wall-clock-paced UDP/TCP race). If set without SITL_LOCKSTEP,
//    rows are keyed to the stock (wall*simRate) clock instead, which
//    isolates clock nondeterminism in A/B experiments.
static bool lockstepEnabled = false;
static uint16_t lockstepRateHz = 250;          // FDM step rate (SITL_LOCKSTEP_RATE_HZ)
static _Atomic uint64_t lockstepVirtualNs = 0; // the one true clock in lockstep mode
static _Atomic bool lockstepEngaged = false;
static uint64_t lockstepBaseNs = 0;            // virtual time at engagement
static double lockstepTs0 = 0.0;               // FDM timestamp at engagement
// Boot "idle warp": before the first motor packet has been sent there is no
// FDM stream to advance the clock (the plugin only streams once it has seen
// a motor packet), and the scheduler will never reach its gyro boundary on a
// frozen clock. So while booting, each pass through the run() idle hook
// advances the virtual clock by the requested idle period -- virtual boot
// time is then a pure function of the scheduler pass count, NOT of wall
// time or host load. The instant the first motor packet leaves (boot
// complete, handshake initiated) the clock freezes until FDM engagement.
static _Atomic bool lockstepBootMotorSent = false;

#define LOCKSTEP_RC_PLAN_MAX_ROWS 512
typedef struct {
    double t;                                   // virtual-clock seconds
    uint16_t ch[SIMULATOR_MAX_RC_CHANNELS];
} lockstepRcRow_t;
static lockstepRcRow_t lockstepRcPlan[LOCKSTEP_RC_PLAN_MAX_ROWS];
static int lockstepRcPlanCount = 0;
static int lockstepRcPlanNext = 0;

static float readRCSITL(const rxRuntimeState_t *rxRuntimeState, uint8_t channel);
static uint8_t rxRCFrameStatus(rxRuntimeState_t *rxRuntimeState);
static void microsleep(uint32_t usec);

static void lockstepLoadRcPlan(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "[SITL][lockstep] cannot open RC plan '%s'\n", path);
        exit(1);
    }
    char line[512];
    while (lockstepRcPlanCount < LOCKSTEP_RC_PLAN_MAX_ROWS && fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        lockstepRcRow_t *row = &lockstepRcPlan[lockstepRcPlanCount];
        for (int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++) {
            row->ch[i] = 1500;
        }
        char *save = NULL;
        char *tok = strtok_r(line, " ,\t", &save);
        if (!tok) {
            continue;
        }
        row->t = atof(tok);
        int i = 0;
        while (i < SIMULATOR_MAX_RC_CHANNELS && (tok = strtok_r(NULL, " ,\t\r\n", &save)) != NULL) {
            row->ch[i++] = (uint16_t)atoi(tok);
        }
        lockstepRcPlanCount++;
    }
    fclose(f);
    printf("[SITL][lockstep] RC plan '%s': %d rows, t=[%.3f..%.3f]\n", path,
           lockstepRcPlanCount,
           lockstepRcPlanCount ? lockstepRcPlan[0].t : 0.0,
           lockstepRcPlanCount ? lockstepRcPlan[lockstepRcPlanCount - 1].t : 0.0);
}

// Apply every plan row that is due at virtual time `nowNs`. Called from the
// FDM thread in updateState() BEFORE the clock is advanced to `nowNs`, so
// the main loop consumes the new channel values exactly at that step.
static void lockstepApplyRcPlan(uint64_t nowNs)
{
    if (lockstepRcPlanNext >= lockstepRcPlanCount) {
        return;
    }
    const double tNow = nowNs * 1e-9;
    bool applied = false;
    while (lockstepRcPlanNext < lockstepRcPlanCount && lockstepRcPlan[lockstepRcPlanNext].t <= tNow) {
        memcpy(rcPkt.channels, lockstepRcPlan[lockstepRcPlanNext].ch, sizeof(rcPkt.channels));
        rcPkt.timestamp = lockstepRcPlan[lockstepRcPlanNext].t;
        lockstepRcPlanNext++;
        applied = true;
    }
    if (applied && !rc_received) {
        // Same provider registration the UDP RC path performs on its first
        // packet (udpRCThread); the plan replaces that wall-clock source.
        rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
        rxRuntimeState.rcReadRawFn = readRCSITL;
        rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;
        rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
        rc_received = true;
        printf("[SITL][lockstep] RC plan source registered at t=%.3f\n", tNow);
    }
}
// ---- end AXIO-LOCKSTEP state -------------------------------------------

int targetParseArgs(int argc, char * argv[])
{
    //The first argument should be target IP.
    if (argc > 1) {
        strcpy(simulator_ip, argv[1]);
    }

    // AXIO-LOCKSTEP: parse the environment as early as possible (this runs
    // first thing in main()), so every later time query sees the mode.
    const char *ls = getenv("SITL_LOCKSTEP");
    if (ls && ls[0] && strcmp(ls, "0") != 0) {
        lockstepEnabled = true;
        printf("[SITL][lockstep] LOCKSTEP MODE ENABLED (SITL_LOCKSTEP=%s)\n", ls);
    }
    const char *plan = getenv("SITL_LOCKSTEP_RC_PLAN");
    if (plan && plan[0]) {
        lockstepLoadRcPlan(plan);
    }
    // AXIO-LOCKSTEP: the FDM step rate (= the honest gyro/PID sample rate,
    // see simulatorLockstepGyroRateHz). Default 250 Hz, the axio-nav world's
    // physics rate; override with SITL_LOCKSTEP_RATE_HZ for other worlds.
    const char *lsRate = getenv("SITL_LOCKSTEP_RATE_HZ");
    if (lsRate && lsRate[0]) {
        lockstepRateHz = (uint16_t)atoi(lsRate);
    }
    if (lockstepEnabled) {
        printf("[SITL][lockstep] gyro/PID timebase: %u Hz (FDM step rate)\n", lockstepRateHz);
    }

    printf("[SITL] The SITL will output to IP %s:%d (Gazebo) and %s:%d (RealFlightBridge)\n",
           simulator_ip, PORT_PWM, simulator_ip, PORT_PWM_RAW);
    return 0;
}

int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void)
{
    // AXIO-LOCKSTEP: only gate the PID loop in lockstep mode. With the flag
    // off this returns 0 ("lock acquired, run the loop"), which is exactly
    // the stock behaviour when SIMULATOR_GYROPID_SYNC is not defined.
    if (!lockstepEnabled) {
        return 0;
    }
    return pthread_mutex_trylock(&mainLoopLock);
}

// AXIO-LOCKSTEP: scheduler hook (see scheduler.c). When true, the realtime
// gyro/filter/PID block parks in its boundary poll loop until the FDM
// stream advances the clock -- exactly one realtime pass per sim step.
// False during boot (the idle warp advances the clock between scheduler
// passes, so the stock boundary logic works) and false in stock mode.
bool simulatorLockstepRealtimeParking(void)
{
    return lockstepEnabled
        && atomic_load_explicit(&lockstepBootMotorSent, memory_order_acquire);
}

// AXIO-LOCKSTEP: the honest sensor/PID sample rate (gyro_sync.c hook).
// In lockstep mode the realtime gyro/filter/PID block runs EXACTLY once
// per FDM step, so the believed sample rate must be the step rate.
// Without this the virtual gyro claims the stock 8 kHz: every dT-derived
// constant (PID I/D scaling, every pt1/biquad filter coefficient, RC
// smoothing, ...) is then computed for 125 us but applied at 4 ms steps
// -- effective filter cutoffs / 32, D-term x 32 -- a sluggish, wrongly
// tuned inner loop that destabilized the axio-nav position cascade in
// closed loop (S1 finding; the spike's open-loop RC-plan hover masked
// it). Returns 0 in stock mode (no override).
uint16_t simulatorLockstepGyroRateHz(void)
{
    return lockstepEnabled ? lockstepRateHz : 0;
}

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

static void sendMotorUpdate(void)
{
    // (Does NOT set lockstepBootMotorSent: this path also serves the
    // stale-FDM "anti-wedge" handshake reply, which can fire while the
    // firmware is still booting. Only a PID-loop motor update proves boot
    // is complete -- see pwmCompleteMotorUpdate().)
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}

// FDM packet bookkeeping, shared between the receive path (udpThread) and
// the apply path (which, in lockstep mode, runs on the MAIN thread -- see
// updateStateApply / simulatorLockstepPassComplete).
static double fdmLastTimestamp = 0;     // [s]
static uint64_t fdmLastRealtimeUs = 0;  // [us] wall
static struct timespec fdmLastTs;       // wall timespec of last applied packet

// AXIO-LOCKSTEP deferred apply: at high real-time factors the next FDM
// packet arrives while the main thread is still draining the non-realtime
// tasks that became due at the previous step. Applying sensors/RC/clock
// from the UDP thread at packet-arrival time would make that race (and so
// the task schedule) wall-clock dependent -- empirically meters of
// run-to-run divergence at ~50x real time. Instead the UDP thread only
// BUFFERS the packet; the main thread applies it at the end of the first
// scheduler pass that executed no task (drain complete), making the whole
// step sequence a pure function of simulation state at any RTF.
static pthread_mutex_t lockstepPendingLock = PTHREAD_MUTEX_INITIALIZER;
static fdm_packet lockstepPendingPkt;
static _Atomic bool lockstepPendingValid = false;

// Applies one FDM packet to the firmware: sensors, (lockstep) RC plan and
// virtual clock. Stock mode: called directly from the UDP thread (status
// quo). Lockstep mode: called only from the main thread.
static void updateStateApply(const fdm_packet* pkt)
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    const double deltaSim = pkt->timestamp - fdmLastTimestamp;  // in seconds
    if (deltaSim < 0) { // don't use old packet
        return;
    }

    int16_t x,y,z;
    x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, -32767, 32767);
    y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, -32767, 32767);
    z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, -32767, 32767);
    virtualAccSet(virtualAccDev, x, y, z);
//    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    // AV fork (axio-nav sim): Gazebo-bridge gyro frame conversion backported
    // from Betaflight master (26524e20d "SITL: Gazebo Harmonic model fixes",
    // gated upstream by ENABLE_GAZEBO_BRIDGE in 035a4d655). The
    // BetaflightPlugin reads angular velocity from the IMU *sensor* entity
    // (components::AngularVelocity on imuEntity), so the data arrives in the
    // sensor frame. The IMU sensor pose is Rx(pi) relative to the FLU link,
    // which makes the sensor frame effectively FRD (X=fwd, Y=right, Z=down).
    //
    // BF gyro conventions:
    //   Roll  = +wx_FRD (roll right)                               -> keep X
    //   Pitch = -wy_FRD (BF positive = nose down, opposite to FRD) -> negate Y
    //   Yaw   = +wz_FRD (CW viewed from above)                     -> keep Z
    //
    // The 2025.12 negation of Z inverted the yaw-rate feedback against this
    // plugin: positive feedback in the yaw loop, observed as the ~300 deg/s
    // uncontrolled yaw spin in axio-nav M3 (docs/sim-stack-spike.md bug 5).
    x = constrain(pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, x, y, z);
//    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

    // temperature in 0.01 C = 25 deg
    // AV fork (axio-nav sim): the aeroloop_gazebo (gz branch) BetaflightPlugin
    // fdmPacket carries no pressure field; the trailing bytes SITL reads as
    // `pressure` are uninitialized plugin stack memory (the plugin's
    // "pkt.escTemperature[4] = {};" zeroes nothing), typically a positive
    // denormal like 6.9e-310. A non-positive/garbage pressure permanently
    // blocks baro calibration (and thus arming), because
    // performBaroCalibrationCycle() is only reached when pressure > 0 and
    // baroInit() ignores baro_hardware=NONE when USE_VIRTUAL_BARO is set.
    // Derive ISA pressure from FDM altitude instead (mirrors what Betaflight
    // master's SITL_GAZEBO config does natively). Treat anything below
    // 1000 Pa as "absent" so FDMs that do send pressure keep working.
    double pressurePa = pkt->pressure;
    if (!(pressurePa > 1000.0)) {
#if defined(USE_VIRTUAL_GPS)
        const double altM = pkt->position_xyz[2]; // [lon, lat, alt] in VIRTUAL_GPS mode
#else
        const double altM = -pkt->position_xyz[2]; // NED down -> altitude
#endif
        pressurePa = 101325.0 * pow(1.0 - 2.25577e-5 * altM, 5.25588);
    }
    virtualBaroSet(pressurePa, 2500);
#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    // AV fork (axio-nav sim): Gazebo-bridge attitude frame conversion
    // backported from Betaflight master (26524e20d / 035a4d655,
    // ENABLE_GAZEBO_BRIDGE). The Gazebo BetaflightPlugin computes the
    // quaternion as Rx(pi)*M*Rx(pi) (a similarity transform), but Betaflight
    // needs the body(FRD)-to-world(NED) quaternion:
    //   ENUtoNED * M * FRDtoFLU = Rz(pi/2) * Rx(pi) * M * Rx(pi).
    // Pre-multiply by Rz(90 deg) to correct: q' = q_Rz(pi/2) * q_plugin.
    // Without this, BF's attitude (and angle mode) is rotated relative to
    // the world, the residual hover drift of docs/sim-stack-spike.md bug 5.
    {
        const float qw = pkt->imu_orientation_quat[0];
        const float qx = pkt->imu_orientation_quat[1];
        const float qy = pkt->imu_orientation_quat[2];
        const float qz = pkt->imu_orientation_quat[3];
        static const float k = 0.70710678f; // cos(pi/4) = sin(pi/4) = sqrt(2)/2
        imuSetAttitudeQuat(k * (qw - qz), k * (qx - qy), k * (qy + qx), k * (qz + qw));
    }
#endif
#endif

#if defined(USE_VIRTUAL_GPS)
    const double longitude = pkt->position_xyz[0];
    const double latitude = pkt->position_xyz[1];
    const double altitude = pkt->position_xyz[2];

    // AV fork (axio-nav sim): GPS mirror backported from Betaflight master
    // (26524e20d / 035a4d655, ENABLE_GAZEBO_BRIDGE). Gazebo Harmonic's
    // SphericalFromLocalPosition inverts horizontal position deltas: moving
    // East in the ENU world frame produces DECREASING longitude, and moving
    // North produces DECREASING latitude. Mirror the GPS position around the
    // initial origin to correct this 180-degree horizontal inversion.
    // Assumes the first FDM packet arrives while the vehicle is at its spawn
    // position; the axio-nav harness starts the world before the SITL, which
    // guarantees this.
    static double originLat = 0, originLon = 0;
    static bool gpsOriginSet = false;
    if (!gpsOriginSet) {
        originLat = latitude;
        originLon = longitude;
        gpsOriginSet = true;
    }
    const double correctedLat = 2.0 * originLat - latitude;
    const double correctedLon = 2.0 * originLon - longitude;

    const double speed = sqrt(sq(pkt->velocity_xyz[0]) + sq(pkt->velocity_xyz[1]));
    const double speed3D = sqrt(sq(pkt->velocity_xyz[0]) + sq(pkt->velocity_xyz[1]) + sq(pkt->velocity_xyz[2]));
    // Plugin provides ENU velocity when spherical coords configured: [0]=East, [1]=North.
    // Course = atan2(East, North) gives standard aviation bearing from North, clockwise.
    double course = atan2(pkt->velocity_xyz[0], pkt->velocity_xyz[1]) * RAD2DEG;
    if (course < 0.0) {
        course += 360.0;
    }
    setVirtualGPS(correctedLat, correctedLon, altitude, speed, speed3D, course);
#endif

#if defined(SIMULATOR_IMU_SYNC)
    imuSetHasNewData(deltaSim*1e6);
    imuUpdateAttitude(micros());
#endif

    if (deltaSim < 0.02 && deltaSim > 0) { // simulator should run faster than 50Hz
//        simRate = simRate * 0.5 + (1e6 * deltaSim / (realtime_now - last_realtime)) * 0.5;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &fdmLastTs);
        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);
    }
//    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);

    fdmLastTimestamp = pkt->timestamp;
    fdmLastRealtimeUs = micros64_real();

    fdmLastTs.tv_sec = now_ts.tv_sec;
    fdmLastTs.tv_nsec = now_ts.tv_nsec;

    // AXIO-LOCKSTEP: compute the virtual time this packet advances us to,
    // and apply any scripted RC rows that become due, BEFORE unlocking the
    // main loop / publishing the new clock value. Ordering is load-bearing
    // for determinism: sensor + RC state must be fully written before the
    // main thread can observe the new time and run the gyro/PID pass.
    uint64_t lockstepNowNs = 0;
    if (lockstepEnabled) {
        if (!atomic_load_explicit(&lockstepEngaged, memory_order_acquire)) {
            lockstepBaseNs = atomic_load_explicit(&lockstepVirtualNs, memory_order_relaxed);
            lockstepTs0 = pkt->timestamp;
            atomic_store_explicit(&lockstepEngaged, true, memory_order_release);
            printf("[SITL][lockstep] engaged: fdm_ts0=%.6f s, virtual_base=%.6f s\n",
                   lockstepTs0, lockstepBaseNs * 1e-9);
        }
        lockstepNowNs = lockstepBaseNs + (uint64_t)llround((pkt->timestamp - lockstepTs0) * 1e9);
        lockstepApplyRcPlan(lockstepNowNs);
    } else if (lockstepRcPlanCount > 0) {
        // Plan without lockstep (A/B isolation experiments): key rows to
        // the stock clock via its cross-thread snapshot.
        lockstepApplyRcPlan(atomic_load_explicit(&microsSnapshot, memory_order_relaxed) * 1000ULL);
    }

    pthread_mutex_unlock(&updateLock); // can send PWM output now

#if defined(SIMULATOR_GYROPID_SYNC)
    if (lockstepEnabled) {
        pthread_mutex_unlock(&mainLoopLock); // can run one main loop iteration
    }
#endif

    if (lockstepEnabled) {
        // Publish the new clock LAST: the next scheduler pass sees the
        // boundary as due and runs exactly one realtime (gyro/PID) block.
        atomic_store_explicit(&lockstepVirtualNs, lockstepNowNs, memory_order_release);
        atomic_store_explicit(&microsSnapshot, lockstepNowNs / 1000, memory_order_relaxed);
    }
}

// Receive path (udpThread).
static void updateState(const fdm_packet* pkt)
{
    const uint64_t realtime_now = micros64_real();
    if (realtime_now > fdmLastRealtimeUs + 500*1e3) { // 500ms timeout
        fdmLastTimestamp = pkt->timestamp;
        fdmLastRealtimeUs = realtime_now;
        // AXIO-LOCKSTEP: once engaged, a long wall-clock gap is just Gazebo
        // having been paused (or running very slowly) -- the virtual clock
        // derives from absolute FDM timestamps and needs no re-baselining,
        // and swallowing the packet here would hold the motors for one
        // extra step (a real divergence seed after pause/resume; measured
        // 1.4 m after 20 s open-loop). Process the packet normally; the
        // gated PID loop produces the motor reply. The unconditional reply
        // below remains for stock mode and for pre-engagement bring-up
        // pokes (anti-wedge).
        if (!lockstepEnabled || !atomic_load_explicit(&lockstepEngaged, memory_order_acquire)) {
            sendMotorUpdate();
            return;
        }
    }

    if (lockstepEnabled) {
        // AXIO-LOCKSTEP: while the firmware is still booting (no PID-loop
        // motor update yet), ignore FDM *content* entirely: sensors, clock
        // and RC plan state must stay a pure function of the deterministic
        // virtual boot timeline, not of how much of the (wall-paced) FDM
        // stream happened to arrive during boot. The stale-FDM handshake
        // reply above still fires, so the plugin can come online while we
        // boot; it re-syncs once the scheduler is running.
        if (!atomic_load_explicit(&lockstepBootMotorSent, memory_order_acquire)) {
            return;
        }
        // Deferred apply (see lockstepPendingLock above): latest packet
        // wins; in a healthy lockstep there is at most one outstanding.
        pthread_mutex_lock(&lockstepPendingLock);
        lockstepPendingPkt = *pkt;
        atomic_store_explicit(&lockstepPendingValid, true, memory_order_release);
        pthread_mutex_unlock(&lockstepPendingLock);
        return;
    }

    updateStateApply(pkt);
}

// AXIO-LOCKSTEP: called from scheduler.c at the end of every scheduler
// pass (sim builds only). Applies the pending FDM packet -- sensors, RC
// plan, clock step -- once the non-realtime task drain is complete (a pass
// that executed no task), from the main thread.
void simulatorLockstepPassComplete(bool ranTask)
{
    if (!lockstepEnabled || ranTask) {
        return;
    }
    if (!atomic_load_explicit(&lockstepPendingValid, memory_order_acquire)) {
        return;
    }
    fdm_packet pkt;
    pthread_mutex_lock(&lockstepPendingLock);
    pkt = lockstepPendingPkt;
    atomic_store_explicit(&lockstepPendingValid, false, memory_order_relaxed);
    pthread_mutex_unlock(&lockstepPendingLock);
    updateStateApply(&pkt);
}

static void* udpThread(void* data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
        if (n == sizeof(fdm_packet)) {
            if (!fdm_received) {
                printf("[SITL] new fdm %d t:%f from %s:%d\n", n, fdmPkt.timestamp, inet_ntoa(stateLink.recv.sin_addr), stateLink.recv.sin_port);
                fdm_received = true;
            }
            updateState(&fdmPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

static float readRCSITL(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return rcPkt.channels[channel];
}

static uint8_t rxRCFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    return RX_FRAME_COMPLETE;
}

static void *udpRCThread(void *data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&rcLink, &rcPkt, sizeof(rc_packet), 100);
        if (n == sizeof(rc_packet)) {
            if (!rc_received) {
                printf("[SITL] new rc %d: t:%f AETR: %d %d %d %d AUX1-4: %d %d %d %d\n", n, rcPkt.timestamp,
                    rcPkt.channels[0], rcPkt.channels[1],rcPkt.channels[2],rcPkt.channels[3],
                    rcPkt.channels[4], rcPkt.channels[5],rcPkt.channels[6],rcPkt.channels[7]);

                rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
                rxRuntimeState.rcReadRawFn = readRCSITL;
                rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;

                rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
                rc_received = true;
            }
        }
    }

    printf("udpRCThread end!!\n");
    return NULL;
}

static void* tcpThread(void* data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.01f);

    // AV fork: tcpServe() = drain all TX rings into dyad + one
    // dyad_update, under the dyad lock -- this thread is the single
    // dyad writer (producers only append to the per-port TX rings; the
    // old producer-side dyad_write raced dyad_update and tore
    // state-link frames, see drivers/serial_tcp.c).
    while (workerRunning) {
        tcpServe();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

// system
void systemInit(void)
{
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // virtual 500MHz

    if (pthread_mutex_init(&updateLock, NULL) != 0) {
        printf("Create updateLock error!\n");
        exit(1);
    }

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
        printf("Create mainLoopLock error!\n");
        exit(1);
    }

    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);

    ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);

    ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);

    ret = udpInit(&rcLink, NULL, PORT_RC, true);
    printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    ret = pthread_create(&udpWorkerRC, NULL, udpRCThread, NULL);
    if (ret != 0) {
        printf("Create udpRCThread error!\n");
        exit(1);
    }
}

void systemReset(void)
{
    printf("[system]Reset!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    pthread_join(udpWorker, NULL);
    exit(0);
}
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);

    printf("[system]ResetToBootloader!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    pthread_join(udpWorker, NULL);
    exit(0);
}

void timerInit(void)
{
    printf("[timer]Init...\n");
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

// (microsSnapshot declaration hoisted above updateState() -- see the
// AXIO-LOCKSTEP block at the top of this file.)

uint64_t micros64(void)
{
    // AXIO-LOCKSTEP: in lockstep mode the clock is the FDM-derived virtual
    // time, frozen between packets. Reads are thread-safe. The main thread
    // parks in the scheduler's boundary poll loop calling this at full
    // speed; back off to 1 us sleeps once the value has provably stopped
    // moving so a waiting (or paused) SITL does not burn a whole core.
    // Sleeping never advances virtual time, so determinism is unaffected.
    if (lockstepEnabled) {
        static uint64_t lastSeenUs = 0;
        static uint32_t sameCount = 0;
        const uint64_t us = atomic_load_explicit(&lockstepVirtualNs, memory_order_acquire) / 1000;
        if (us == lastSeenUs) {
            // High threshold so short between-step task drains are never
            // throttled; only a genuinely waiting/paused SITL backs off.
            if (++sameCount > 1024) {
                microsleep(1);
                // Freeze diagnostic (wedge triage): a clock frozen for
                // seconds of WALL time is either a paused Gazebo /
                // offline plugin (pending_fdm=0: nothing to apply) or a
                // stuck deferred apply (pending_fdm=1: a buffered packet
                // the scheduler never gets to drain). Logged at most
                // every 2 s, main thread only.
                static uint64_t lastFreezeLogUs = 0;
                const uint64_t wallUs = micros64_real();
                if (sameCount > 200000 && wallUs - lastFreezeLogUs > 2000000) {
                    lastFreezeLogUs = wallUs;
                    fprintf(stderr, "[SITL][lockstep] clock frozen at %llu us (pending_fdm=%d)\n",
                            (unsigned long long)us,
                            atomic_load_explicit(&lockstepPendingValid, memory_order_relaxed) ? 1 : 0);
                }
            }
        } else {
            lastSeenUs = us;
            sameCount = 0;
        }
        return us;
    }

    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    const uint64_t us = out / 1000;
    atomic_store_explicit(&microsSnapshot, us, memory_order_relaxed);
    return us;
}

uint64_t millis64(void)
{
    // AXIO-LOCKSTEP: see micros64().
    if (lockstepEnabled) {
        return atomic_load_explicit(&lockstepVirtualNs, memory_order_acquire) / (1000 * 1000);
    }

    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out / (1000 * 1000);
}

uint32_t micros(void)
{
    return micros64() & 0xFFFFFFFF;
}

// AV fork: ISR-context time. Serial RX providers (CRSF) timestamp received
// bytes with microsISR(); on hardware it reads the cycle counter safely
// from interrupt context. In SITL the "ISR" is the dyad TCP thread, which
// must not advance the (thread-unsafe) sim clock -- return the main loop's
// latest micros64() snapshot instead.
uint32_t microsISR(void)
{
    return (uint32_t)atomic_load_explicit(&microsSnapshot, memory_order_relaxed);
}

uint32_t millis(void)
{
    return millis64() & 0xFFFFFFFF;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return clockCycles;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros;
}

uint32_t getCycleCounter(void)
{
    return (uint32_t) (micros64() & 0xFFFFFFFF);
}

static void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us)
{
    // AXIO-LOCKSTEP: during boot, sleeping IS what advances the virtual
    // clock -- deterministically and instantly. Once the scheduler is alive
    // the clock belongs to the FDM stream; sleep in (unscaled) wall time.
    if (lockstepEnabled) {
        if (!atomic_load_explicit(&lockstepBootMotorSent, memory_order_acquire)) {
            atomic_fetch_add_explicit(&lockstepVirtualNs, (uint64_t)us * 1000ULL, memory_order_relaxed);
            return;
        }
        microsleep(us);
        return;
    }
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us)
{
    // AXIO-LOCKSTEP: the only in-tree caller is the run() idle loop
    // (RUN_LOOP_DELAY_US = 50).
    if (lockstepEnabled) {
        // Boot idle warp (see lockstepBootMotorSent): advance virtual time
        // deterministically until the first PID-loop motor packet initiates
        // the FDM handshake.
        if (!atomic_load_explicit(&lockstepBootMotorSent, memory_order_acquire)) {
            atomic_fetch_add_explicit(&lockstepVirtualNs, (uint64_t)us * 1000ULL, memory_order_relaxed);
            return;
        }
        // Engaged (or waiting for engagement): the scheduler must react to
        // FDM packets as fast as possible -- the step round-trip is the
        // FTRT ceiling, and nanosleep() granularity is ~60 us regardless
        // of the requested time, which would dominate it. Skip the idle
        // sleep entirely; the frozen-clock backoff in micros64() throttles
        // a genuinely idle (waiting/paused) SITL instead. Wall sleeping
        // never advances sim time, so this is a latency/CPU trade only.
        return;
    }
    microsleep(us);
}

void delay(uint32_t ms)
{
    // AXIO-LOCKSTEP: during boot, advance the virtual clock directly
    // (see delayMicroseconds). Afterwards, fall through to the stock
    // spin: millis64() advances as FDM packets arrive. NOTE: if Gazebo is
    // paused while the firmware sits in delay(), it stays here -- that is
    // the lockstep contract (sim time is the only time).
    if (lockstepEnabled && !atomic_load_explicit(&lockstepBootMotorSent, memory_order_acquire)) {
        atomic_fetch_add_explicit(&lockstepVirtualNs, (uint64_t)ms * 1000000ULL, memory_order_relaxed);
        return;
    }

    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y)
{
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}

// PWM part
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    printf("[SITL] Init servos num %d rate %d center %d\n", MAX_SUPPORTED_SERVOS,
           servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    // NOOP
}

static void pwmWriteMotor(uint8_t index, float value)
{
    if (pthread_mutex_trylock(&updateLock) != 0) return;

    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = value - idlePulse;
    }

    if (index < pwmRawPkt.motorCount) {
        pwmRawPkt.pwm_output_raw[index] = value;
    }

    pthread_mutex_unlock(&updateLock); // can send PWM output now
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    // NOOP
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!
    if (pthread_mutex_trylock(&updateLock) != 0) return;
    // AXIO-LOCKSTEP: a PID-loop motor update means init is over and the
    // scheduler is alive -- stop the boot idle warp and allow lockstep
    // engagement (set BEFORE the send so the FDM reply can never race the
    // flag).
    atomic_store_explicit(&lockstepBootMotorSent, true, memory_order_release);
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
//    printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(servo_packet_raw));
}

void servoWrite(uint8_t index, float value)
{
    servosPwm[index] = value;
    if (index + pwmRawPkt.motorCount < SIMULATOR_MAX_PWM_CHANNELS) {
        // In pwmRawPkt, we put servo right after the motors.
        pwmRawPkt.pwm_output_raw[index + pwmRawPkt.motorCount] = value;
    }
}

static const motorVTable_t vTable = {
    .postInit = motorPostInitNull,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .decodeTelemetry = motorDecodeTelemetryNull,
    .write = pwmWriteMotor,
    .writeInt = pwmWriteMotorInt,
    .updateComplete = pwmCompleteMotorUpdate,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t _idlePulse)
{
    UNUSED(motorConfig);

    if (!device) {
        return false;
    }

    pwmMotorCount = device->count;
    device->vTable = &vTable;

    printf("Initialized motor count %d\n", pwmMotorCount);
    pwmRawPkt.motorCount = pwmMotorCount;

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {
        pwmMotors[motorIndex].enabled = true;
    }

    return true;
}

// stack part
char _estack;
char _Min_Stack_Size;

// virtual EEPROM
static FILE *eepromFd = NULL;

bool loadEEPROMFromFile(void)
{
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return false;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME, "r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return false;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return false;
        }

        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
            return false;
        }
    }
    return true;
}

void configUnlock(void)
{
    loadEEPROMFromFile();
}

void configLock(void)
{
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t), "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    if ((address >= (uintptr_t)eepromData) && (address + sizeof(uint32_t) <= (uintptr_t)ARRAYEND(eepromData))) {
        memcpy((void*)address, buffer, sizeof(config_streamer_buffer_type_t));
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)address, *((uint32_t*)address));
    } else {
        printf("[FLASH_ProgramWord]%p out of range!\n", (void*)address);
    }
    return CONFIG_RESULT_SUCCESS;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    printf("IOConfigGPIO\n");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind\n");
}

void debugInit(void)
{
    printf("debugInit\n");
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}

void IOHi(IO_t io)
{
    UNUSED(io);
}

void IOLo(IO_t io)
{
    UNUSED(io);
}

void IOInitGlobal(void)
{
    // NOOP
}

IO_t IOGetByTag(ioTag_t tag)
{
    UNUSED(tag);
    return NULL;
}

bool usbCableIsInserted(void)
{
    return false;
}

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = { .id = MCU_TYPE_SIMULATOR, .name = "SIMULATOR" };
    return &info;
}
