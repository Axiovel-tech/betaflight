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
 * One-way, fixed-size, CRC-framed binary state stream from Betaflight to
 * axio-nav on a dedicated serial port, at a CLI-configurable rate.
 *
 * Specification (single source of truth, owned by axio-nav):
 *   axio-nav repo, docs/state-link-protocol.md, protocol version 1.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

#define STATE_LINK_PROTOCOL_VERSION    1

#define STATE_LINK_SYNC                0xA5
#define STATE_LINK_FRAME_TYPE_STATE    0x01

#define STATE_LINK_HEADER_SIZE         4
#define STATE_LINK_STATE_PAYLOAD_SIZE  58
#define STATE_LINK_STATE_FRAME_SIZE    (STATE_LINK_HEADER_SIZE + STATE_LINK_STATE_PAYLOAD_SIZE + 1)

#define STATE_LINK_RATE_HZ_MIN         100
#define STATE_LINK_RATE_HZ_MAX         1000
#define STATE_LINK_RATE_HZ_DEFAULT     500

#define STATE_LINK_FLAG_ARMED          (1 << 0)
#define STATE_LINK_FLAG_FAILSAFE       (1 << 1)
#define STATE_LINK_FLAG_CALIBRATING    (1 << 2)
#define STATE_LINK_FLAG_MSP_OVERRIDE   (1 << 3)  // BOXMSPOVERRIDE active: axio-nav has stick control

typedef struct stateLinkConfig_s {
    uint16_t rate_hz;       // STATE frame rate [Hz]
} stateLinkConfig_t;

PG_DECLARE(stateLinkConfig_t, stateLinkConfig);

// In-memory image of one STATE v1 frame, in wire units and wire frame
// conventions (body FRD, world NED, SI, radians).
typedef struct stateLinkStateFrame_s {
    uint32_t timestampUs;   // micros() at sampling
    float gyroRadS[3];      // body angular rate, FRD [rad/s]
    float accelMps2[3];     // specific force, body FRD [m/s^2]
    float quat[4];          // attitude quaternion w,x,y,z (Hamilton, FRD -> NED)
    uint16_t motor[4];      // normalized motor outputs, 0..2047
    uint16_t vbatMv;        // battery voltage [mV]
    uint16_t currentCa;     // battery current [10 mA]
    uint8_t flags;          // STATE_LINK_FLAG_*
    uint8_t seq;            // frame sequence counter
} stateLinkStateFrame_t;

// Sample the firmware state into a frame struct (wire units/conventions).
void stateLinkSampleState(stateLinkStateFrame_t *frame, timeUs_t currentTimeUs);

// Serialize a frame struct into dst (>= STATE_LINK_STATE_FRAME_SIZE bytes).
// Returns the frame size in bytes.
int stateLinkSerializeStateFrame(uint8_t *dst, const stateLinkStateFrame_t *frame);

void stateLinkInit(void);
bool stateLinkIsEnabled(void);
void stateLinkProcess(timeUs_t currentTimeUs);
