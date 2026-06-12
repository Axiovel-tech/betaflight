/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * axio-nav state link (Axiovel fork addition).
 *
 * Implements protocol version 1 of the axio-nav state-link specification
 * (axio-nav repo, docs/state-link-protocol.md): a one-way stream of
 * fixed-size, CRC-8/DVB-S2 framed STATE records on a dedicated serial
 * port (FUNCTION_STATE_LINK), at `state_link_rate_hz` (100-1000 Hz).
 *
 * Wire conventions are body FRD / world NED (SI, radians). Betaflight's
 * internal conventions are body FLU (x fwd, y left, z up) and world NWU
 * with its attitude quaternion mapping body FLU -> world NWU (see
 * flight/imu.c). Both frame changes are the same 180-degree rotation
 * about x, so the conversions used here are:
 *
 *   gyro/accel:  (x, y, z)_FRD = (x, -y, -z)_BF
 *   quaternion:  q_FRD->NED    = (w, x, -y, -z)_BF   (hardware/Mahony)
 *
 * EXCEPTION (SITL): when the attitude is injected by the FDM
 * (imuSetAttitudeQuat path in platform/SIMULATOR/sitl.c, i.e.
 * SIMULATOR_BUILD without USE_IMU_CALC/SET_IMU_FROM_EULER), the internal
 * quaternion is already body FRD -> world NED -- the same convention
 * flight/imu.c compensates for with its simulator-only rMat sign flips.
 * The transmitter passes it through unchanged in that configuration; see
 * stateLinkSampleState().
 *
 * Data taps (provisional per spec, to be confirmed by the M2 bench
 * campaign):
 *   - gyro:  gyro.gyroADCf [deg/s] -- after the full gyro filter chain
 *            (RPM filter, static notches, gyro lowpass 1, dynamic notch;
 *            gyro lowpass 2 is applied upstream as the decimation filter).
 *            NOTE: the spec describes this tap as "post gyro lowpass,
 *            before RPM/dynamic notch"; no such intermediate signal
 *            exists in Betaflight 2025.12 -- gyroADCf is post-all-filters.
 *   - accel: acc.accADC [LSB, acc.dev.acc_1G per g] -- aligned and
 *            trim-corrected accelerometer sample.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_STATE_LINK

#include "common/axis.h"
#include "common/crc.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/state_link.h"

#define STATE_LINK_GRAVITY_MSS          9.80665f
#define STATE_LINK_MOTOR_SCALE_MAX      2047
#define STATE_LINK_BAUD_DEFAULT         BAUD_921600

PG_REGISTER_WITH_RESET_TEMPLATE(stateLinkConfig_t, stateLinkConfig, PG_STATE_LINK_CONFIG, 0);

PG_RESET_TEMPLATE(stateLinkConfig_t, stateLinkConfig,
    .rate_hz = STATE_LINK_RATE_HZ_DEFAULT,
);

static serialPort_t *stateLinkPort = NULL;
static uint8_t stateLinkSeq = 0;

static void putU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)(value >> 8);
}

static void putU32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)((value >> 16) & 0xFF);
    dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void putF32(uint8_t *dst, float value)
{
    // IEEE-754 single precision, little-endian on the wire
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    putU32(dst, bits);
}

void stateLinkSampleState(stateLinkStateFrame_t *frame, timeUs_t currentTimeUs)
{
    frame->timestampUs = (uint32_t)currentTimeUs;

    // Body FLU -> body FRD: negate y and z
    frame->gyroRadS[0] = DEGREES_TO_RADIANS(gyro.gyroADCf[X]);
    frame->gyroRadS[1] = -DEGREES_TO_RADIANS(gyro.gyroADCf[Y]);
    frame->gyroRadS[2] = -DEGREES_TO_RADIANS(gyro.gyroADCf[Z]);

    const float accScale = acc.dev.acc_1G_rec * STATE_LINK_GRAVITY_MSS;
    frame->accelMps2[0] = acc.accADC.x * accScale;
    frame->accelMps2[1] = -acc.accADC.y * accScale;
    frame->accelMps2[2] = -acc.accADC.z * accScale;

    // Betaflight attitude quaternion -> wire body FRD -> world NED.
    //
    // Two cases, matching exactly the build condition flight/imu.c uses
    // for its simulator-only rMat sign compensation:
    //
    //  - Hardware (Mahony, flight/imu.c): the internal quaternion is the
    //    body FLU -> world NWU attitude; FRD -> NED is the Rx(pi)
    //    similarity, i.e. negate the y and z components.
    //
    //  - SITL with an FDM-injected attitude (imuSetAttitudeQuat in
    //    platform/SIMULATOR/sitl.c): the injected quaternion is already
    //    the body FRD -> world NED attitude (that is why imu.c flips
    //    rMat[1][0]/rMat[2][0] under this same condition to recover the
    //    native Euler readouts). Pass it through unchanged. The previous
    //    unconditional negation double-converted here, putting
    //    nose-down-positive pitch and mirrored yaw on the wire
    //    (axio-nav PR-8 wobble-diagnosis side finding, verified against
    //    horizontal acceleration on the M5 baseline).
    quaternion_t q;
    getQuaternion(&q);
#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    frame->quat[0] = q.w;
    frame->quat[1] = q.x;
    frame->quat[2] = q.y;
    frame->quat[3] = q.z;
#else
    frame->quat[0] = q.w;
    frame->quat[1] = q.x;
    frame->quat[2] = -q.y;
    frame->quat[3] = -q.z;
#endif

    // Post-mixer motor outputs, normalized to 0..2047 over the protocol
    // (DShot or PWM) output range. motorConvertToExternal() maps the
    // protocol range to the external 1000..2000 range for all protocols.
    const int motorCount = MIN(getMotorCount(), 4);
    for (int i = 0; i < 4; i++) {
        if (i < motorCount) {
            const int external = constrain(motorConvertToExternal(motor[i]), PWM_RANGE_MIN, PWM_RANGE_MAX);
            frame->motor[i] = ((external - PWM_RANGE_MIN) * STATE_LINK_MOTOR_SCALE_MAX
                               + (PWM_RANGE_MAX - PWM_RANGE_MIN) / 2) / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        } else {
            frame->motor[i] = 0;
        }
    }

    // Betaflight reports voltage in 10 mV steps, the wire wants mV
    frame->vbatMv = (uint16_t)MIN((uint32_t)getBatteryVoltage() * 10, (uint32_t)UINT16_MAX);
    frame->currentCa = (uint16_t)constrain(getAmperage(), 0, UINT16_MAX);

    uint8_t flags = 0;
    if (ARMING_FLAG(ARMED)) {
        flags |= STATE_LINK_FLAG_ARMED;
    }
    if (failsafeIsActive()) {
        flags |= STATE_LINK_FLAG_FAILSAFE;
    }
    if (!gyroIsCalibrationComplete()
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())) {
        flags |= STATE_LINK_FLAG_CALIBRATING;
    }
    frame->flags = flags;

    frame->seq = stateLinkSeq;
}

int stateLinkSerializeStateFrame(uint8_t *dst, const stateLinkStateFrame_t *frame)
{
    // header
    dst[0] = STATE_LINK_SYNC;
    dst[1] = STATE_LINK_PROTOCOL_VERSION;
    dst[2] = STATE_LINK_FRAME_TYPE_STATE;
    dst[3] = STATE_LINK_STATE_PAYLOAD_SIZE;

    // payload
    uint8_t *p = &dst[STATE_LINK_HEADER_SIZE];
    putU32(&p[0], frame->timestampUs);
    for (int i = 0; i < 3; i++) {
        putF32(&p[4 + 4 * i], frame->gyroRadS[i]);
        putF32(&p[16 + 4 * i], frame->accelMps2[i]);
    }
    for (int i = 0; i < 4; i++) {
        putF32(&p[28 + 4 * i], frame->quat[i]);
        putU16(&p[44 + 2 * i], frame->motor[i]);
    }
    putU16(&p[52], frame->vbatMv);
    putU16(&p[54], frame->currentCa);
    p[56] = frame->flags;
    p[57] = frame->seq;

    // CRC-8/DVB-S2 over version..payload (sync byte excluded)
    dst[STATE_LINK_STATE_FRAME_SIZE - 1] =
        crc8_dvb_s2_update(0, &dst[1], STATE_LINK_STATE_FRAME_SIZE - 2);

    return STATE_LINK_STATE_FRAME_SIZE;
}

void stateLinkInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_STATE_LINK);
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = STATE_LINK_BAUD_DEFAULT;
    }

    // Dedicated, one-way (TX only), never shared with another function
    stateLinkPort = openSerialPort(portConfig->identifier, FUNCTION_STATE_LINK,
                                   NULL, NULL, baudRates[baudRateIndex],
                                   MODE_TX, SERIAL_NOT_INVERTED);
}

bool stateLinkIsEnabled(void)
{
    return stateLinkPort != NULL;
}

void stateLinkProcess(timeUs_t currentTimeUs)
{
    if (!stateLinkPort) {
        return;
    }

    stateLinkStateFrame_t frame;
    stateLinkSampleState(&frame, currentTimeUs);

    uint8_t buf[STATE_LINK_STATE_FRAME_SIZE];
    const int length = stateLinkSerializeStateFrame(buf, &frame);

#ifdef SIMULATOR_BUILD
    // SITL serves serial ports as TCP servers (drivers/serial_tcp.c). The
    // driver's batched writeBuf (AV fork, tcpWriteBuf) appends the whole
    // frame to the TX ring and flushes it with ONE dyad_write: never
    // blocking (before a client attaches the ring just wraps and the
    // receiver resynchronizes via sync hunt + CRC), and it keeps the
    // frame contiguous against the dyad thread -- the previous per-byte
    // serialWrite loop made 62 racy dyad_write calls per frame (observed
    // as rare duplicated/torn frames at 250 Hz; tcpWriteBuf comment).
    serialWriteBuf(stateLinkPort, buf, length);
#else
    // Never block: if the TX buffer can't take a full frame, skip this
    // cycle. seq only advances on frames actually queued, so a skipped
    // frame does not appear as wire loss to the receiver.
    if (serialTxBytesFree(stateLinkPort) < (uint32_t)length) {
        return;
    }
    serialWriteBuf(stateLinkPort, buf, length);
#endif

    stateLinkSeq++;
}

#endif // USE_TELEMETRY_STATE_LINK
