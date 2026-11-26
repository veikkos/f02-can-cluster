#include "config.h"
#include <Arduino.h>
#include "types.h"
#include "serial.h"
#include "pc_printf.h"
#include "ad5272_ambient.h"
#include "can_adapter.h"

/*
    BMW F-series instrument cluster (F01/F02) on a 500 kbit/s CAN bus. See config.h for options.

    Many frames carry a CRC and an alive counter:
      byte 0  CRC8 (polynomial 0x1D, init 0xFF) over the rest of the frame, XOR a constant per ID
      byte 1  low nibble is an alive counter 0..14
*/

#if defined(USE_AD5272_AMBIENT)
    AD5272Ambient ambientTemp;
#endif

STimers s_timers;
SInput s_input;

static uint8_t crc8(const uint8_t* data, uint8_t len, uint8_t xor_out) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
        }
    }
    return crc ^ xor_out;
}

static void canSendCrc(uint32_t id, uint8_t* frame, uint8_t len, uint8_t xor_out) {
    frame[0] = crc8(&frame[1], len - 1, xor_out);
    canSend(id, frame, len);
}

static inline uint8_t nextCounter(uint8_t& counter) {
    uint8_t value = counter;
    counter = (counter + 1) % 15;
    return value;
}

bool canSendIgnitionFrame() {
    static uint8_t counter = 0;

    // Terminal state: 0x06 KL30B (display off), 0x08 KL R, 0x8A KL15
    uint8_t state;
    switch (s_input.ignition) {
        case IG_ACCESSORY: state = 0x08; break;
        case IG_ON:
        case IG_STARTER:   state = 0x8A; break;
        case IG_OFF:
        default:           state = 0x06; break;
    }

    uint8_t frame[8] = {0, (uint8_t)(0x80 | nextCounter(counter)), state, 0xDD, 0xF1, 0x01, 0x30, 0x06};
    canSendCrc(0x12F, frame, sizeof(frame), 0x44);
    return true;
}

// Network management frame of the gateway. Keeps the cluster awake with the ignition off.
// Once asleep it cannot be woken from the CAN bus, only by a power cycle.
bool canSendNetworkManagement() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x10};
    canSend(0x510, frame, sizeof(frame));
    return true;
}

// RPM in 10 rpm steps. Byte 4 lights the EfficientDynamics energy recovery
// (from fatihcesur/bmw-f30-cluster-simhub).
bool canSendRPM() {
    static uint8_t counter = 0;

    uint16_t rpm_raw = s_input.rpm / 10;
    if (rpm_raw > 0xFFF) rpm_raw = 0xFFF;

    uint8_t frame[8] = {
        0,
        (uint8_t)(((rpm_raw & 0x0F) << 4) | nextCounter(counter)),
        (uint8_t)(rpm_raw >> 4),
        0xC0,
        (uint8_t)(s_input.coasting ? 0xF6 : 0xF0),
        0xFF, 0xFF, 0xFF
    };
    canSendCrc(0x0F3, frame, sizeof(frame), 0x7A);
    return true;
}

// Accelerator pedal. In ECO PRO a changing value shows the "ease off the pedal" tip,
// so it could be fed from the game's throttle.
bool canSendAcceleratorPedal() {
    static uint8_t counter = 0;
    // 0xC0 is the value in the F11 frame the payload comes from. The cluster takes any
    // value 0x00-0xFF, the scale is not known.
    uint8_t accelerator_pedal = 0xC0;

    uint8_t frame[8] = {
        0,
        (uint8_t)(0x10 | nextCounter(counter)),
        0x00, 0x10, 0x00, 0xE0, 0x7F,
        accelerator_pedal
    };
    canSendCrc(0x0D9, frame, sizeof(frame), 0x22);
    return true;
}

bool canSendSpeed() {
    static uint8_t counter = 0;

    // The cluster shows 10 %, at most 1 km/h, more than it gets
    uint32_t speed = s_input.speed; // km/h * 10
    speed = speed <= 110 ? speed * 10 / 11 : speed - 10;
    uint32_t raw = speed * 64 / 10;
    if (raw > 0xFEFF) raw = 0xFEFF;

    // Direction: 0x81 stationary, 0x91 forward, 0xA1 reverse
    uint8_t state = 0x81;
    if (raw > 0) {
        state = s_input.currentGear == REVERSE ? 0xA1 : 0x91;
    }

    uint8_t frame[5] = {
        0,
        (uint8_t)(0xC0 | nextCounter(counter)),
        (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8),
        state
    };
    canSendCrc(0x1A1, frame, sizeof(frame), 0xA9);
    return true;
}

bool canSendGearboxData() {
    static uint8_t counter = 0;

    // Bits 5-7 position, bits 0-2 program (1 = S, 2 = M)
    uint8_t position = 0x00;
    switch (s_input.currentGear) {
        case PARK:    position = 0x20; break;
        case REVERSE: position = 0x40; break;
        case NEUTRAL: position = 0x60; break;
        case DRIVE:   position = 0x80; break;
    }

    uint8_t gear_number = 0;
    if (s_input.currentGear == DRIVE) {
        if (s_input.mode == SPORT) {
            position |= 0x01;
            gear_number = s_input.explicitGear;
        } else if (s_input.explicitGear != NONE) {
            position |= 0x02;
            gear_number = s_input.explicitGear;
        }
    }

    uint8_t frame[5] = {
        0,
        (uint8_t)((gear_number << 4) | nextCounter(counter)),
        position, 0xFC, 0xFF
    };
    canSendCrc(0x3FD, frame, sizeof(frame), 0xD6);
    return true;
}

bool canSendEngineTemperatures() {
    static uint8_t counter = 0;

    uint8_t frame[8] = {
        0,
        (uint8_t)(0x10 | nextCounter(counter)),
        0x82, 0x4E,
        (uint8_t)min(s_input.water_temp + 48, 0xFE),
        (uint8_t)min(s_input.oil_temp + 48, 0xFE),
        0x05,
        0x89 // Max engine speed / 50
    };
    canSendCrc(0x3F9, frame, sizeof(frame), 0xF1);
    return true;
}

// Frames from other control units. Without them the cluster shows warnings.
bool canSendAbs() {
    static uint8_t counter = 0;
    uint8_t frame[5] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xFE, 0xFF, 0x14};
    canSendCrc(0x36E, frame, sizeof(frame), 0xD8);
    return true;
}

bool canSendPowerSteering() {
    static uint8_t counter = 0;
    uint8_t frame[5] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xFE, 0xFF, 0x14};
    canSendCrc(0x2A7, frame, sizeof(frame), 0x9E);
    return true;
}

// Level control status. Without it, or with a counter that stops, "Chassis" warning.
bool canSendLevelControl() {
    static uint8_t counter = 0;
    uint8_t frame[8] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSendCrc(0x26A, frame, sizeof(frame), 0x00);
    return true;
}

// Emergency call status. Without it, or with a counter that stops, "SOS call" warning.
bool canSendEmergencyCall() {
    static uint8_t counter = 0;
    uint8_t frame[8] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSendCrc(0x2C3, frame, sizeof(frame), 0x00);
    return true;
}

// Gateway alive, the low nibble of byte 0 is the counter
bool canSendGatewayAlive() {
    static uint8_t counter = 0;
    uint8_t frame[2] = {(uint8_t)(0xF0 | nextCounter(counter)), 0xFF};
    canSend(0x0C0, frame, sizeof(frame));
    return true;
}

// Engine torque. Payload from the F11, only keeps the cluster from storing it missing.
bool canSendEngineTorque() {
    static uint8_t counter = 0;
    uint8_t frame[8] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xC6, 0x67, 0x7C, 0x00, 0x00, 0xF1};
    canSendCrc(0x0A5, frame, sizeof(frame), 0x9F);
    return true;
}

// Central locking and lids, no CRC. Payload from the i3: doors closed. The door symbols
// come from check controls, see canSendDoorOpenFrontLeft.
bool canSendCentralLocking() {
    uint8_t frame[8] = {0x81, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSend(0x2FC, frame, sizeof(frame));
    return true;
}

// The next four only keep the cluster from storing them missing. No CRC, all zeros is
// accepted and shows nothing.
bool canSendDscStatus() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x173, frame, sizeof(frame));
    return true;
}

bool canSendDrivingLightsStatus() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x314, frame, sizeof(frame));
    return true;
}

bool canSendSpeedConversionFactor() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x3CB, frame, sizeof(frame));
    return true;
}

// Driving dynamics switch configuration. 0xFF bytes make it invalid.
bool canSendDrivingDynamicsSwitchConfig() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x3E6, frame, sizeof(frame));
    return true;
}

// Vehicle state, no CRC. Byte 2 low nibble 0 is energy state good.
bool canSendVehicleState() {
    uint8_t frame[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x3A0, frame, sizeof(frame));
    return true;
}

// Ambient brightness, no CRC. When dark (byte 0 below about 0x0C) the display follows the
// headlights: night design with the lights on, day design with them off. Bright keeps day design.
bool canSendAmbientBrightness() {
    static bool day_design = false;

    uint8_t frame[8] = {(uint8_t)(day_design ? 0x30 : 0x00), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(0x2A5, frame, sizeof(frame));
    return true;
}

// High beam assistant status, no CRC. Byte 0 bit 1 shows the green "A", 0xFF is invalid.
bool canSendHighBeamAssist() {
    static bool high_beam_assist = false;

    uint8_t frame[8] = {(uint8_t)(high_beam_assist ? 0x02 : 0x01), 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
    canSend(0x36A, frame, sizeof(frame));
    return true;
}

// Cruise control (layout from fatihcesur/bmw-f30-cluster-simhub):
//   byte 2     0x22 set speed marker, 0x26 marker and the set speed as a number
//   byte 5     0x96 cruise symbol, 0x80 in limiter mode, bit 1 makes LIM blink
//   bytes 6-7  set speed * 16, the low two bits are the mode:
//              0 cruise, 1 off, 2 limiter (LIM and the limit), 3 "Cruise control failure"
bool canSendCruiseControl() {
    static bool limiter = false;
    static bool limit_exceeded = false;
    static bool failure = false;

    static uint8_t counter = 0;
    static uint16_t last_speed = 0;
    static uint32_t changed_ms = 0;
    static bool show_number = false;

    // The number is shown for a few seconds after a change, as in the car
    uint16_t speed = s_input.cruise.enabled ? min(s_input.cruise.speed, (uint16_t)300) : 0;
    if (speed != last_speed) {
        last_speed = speed;
        changed_ms = millis();
        show_number = speed > 0;
    }
    if (show_number && millis() - changed_ms >= 3000) show_number = false;

    uint8_t marker = 0xE0;
    uint8_t symbol = 0x00;
    uint16_t set_speed = 0x01EC;
    uint8_t mode = 0;
    if (s_input.cruise.enabled) {
        symbol = limiter ? (limit_exceeded ? 0x82 : 0x80) : 0x96;
        if (limiter) mode = 2;
        if (speed > 0) {
            marker = show_number ? 0x26 : 0x22;
            set_speed = speed * 16;
        }
    }
    if (failure) mode = 3;
    set_speed = (set_speed & ~0x03) | mode;

    uint8_t frame[8] = {
        0,
        (uint8_t)(0xF0 | nextCounter(counter)),
        marker, 0xE0, 0xE1, symbol,
        (uint8_t)(set_speed & 0xFF), (uint8_t)(set_speed >> 8)
    };
    canSendCrc(0x289, frame, sizeof(frame), 0x82);
    return true;
}

bool canSendAirbag() {
    static uint8_t counter = 0;
    uint8_t frame[8] = {0, (uint8_t)(0x40 | nextCounter(counter)), 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF};
    canSendCrc(0x19B, frame, sizeof(frame), 0xFF);
    return true;
}

bool canSendSeatbelt() {
    static uint8_t counter = 0;
    uint8_t frame[7] = {0, (uint8_t)(0xE0 | nextCounter(counter)), 0xF1, 0xF0, 0xF2, 0xF2, 0xFE};
    canSendCrc(0x297, frame, sizeof(frame), 0x28);
    return true;
}

bool canSendTirePressure() {
    static uint8_t counter = 0;
    uint8_t frame[5] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0xA2, 0xA0, 0xA0};
    canSendCrc(0x369, frame, sizeof(frame), 0xC5);
    return true;
}

bool canSendAcc() {
    static uint8_t counter = 0;
    uint8_t frame[6] = {0, (uint8_t)(0xF0 | counter), 0x5C, 0x70, 0x00, 0x00};
    canSendCrc(0x33B, frame, sizeof(frame), 0x6B);
    counter += 4;
    if (counter > 0x0E) counter -= 0x0F;
    return true;
}

bool canSendSafetyCounter() {
    static uint8_t counter = 0;
    uint8_t frame[2] = {counter, 0xFF};
    canSend(0x0D7, frame, sizeof(frame));
    if (++counter >= 254) counter = 0;
    return true;
}

bool canSendHandbrake() {
    static uint8_t counter = 0;
    uint8_t frame[5] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0x38, 0x00,
                        (uint8_t)(s_input.handbrake ? 0x15 : 0x14)};
    canSendCrc(0x36F, frame, sizeof(frame), 0x17);
    return true;
}

bool canSendIndicator() {
    uint8_t frame[2] = {0x80, 0xF0};

    switch (s_input.indicator_state) {
        case I_LEFT:    frame[0] = 0x91; break;
        case I_RIGHT:   frame[0] = 0xA1; break;
        case I_HAZZARD: frame[0] = 0xB1; break;
        case I_OFF:
        default:        frame[0] = 0x80; break;
    }
    canSend(0x1F6, frame, sizeof(frame));
    return true;
}

bool canSendLights() {
    static uint8_t last_value = 0xFF;
    static uint8_t counter = 0;
    counter++;

    uint8_t lights = 0;
    if (s_input.light_lowbeam || s_input.light_highbeam) lights |= L_LOW_BEAM;
    if (s_input.light_highbeam) lights |= L_HIGH_BEAM;
    if (s_input.light_fog) lights |= L_FRONT_FOG | L_REAR_FOG;

    if (lights != last_value || counter % 20 == 0) {
        last_value = lights;
        counter = 0;
        uint8_t frame[3] = {lights, 0xC0, 0xF7};
        canSend(0x21A, frame, sizeof(frame));
        return true;
    }
    return false;
}

// Dial lighting, 0x00 dim to 0xFD bright. The dials cannot be turned off.
bool canSendBacklight() {
    uint8_t frame[2] = {(uint8_t)(s_input.ignition == IG_OFF ? 0x00 : 0xFD), 0xFF};
    canSend(0x202, frame, sizeof(frame));
    return true;
}

bool canSendDriveMode() {
    static uint8_t counter = 0;

    // 1 TRACTION, 2 COMFORT+, 4 SPORT, 5 SPORT+, 6 DSC OFF, 7 ECO PRO
    uint8_t mode;
    switch (s_input.drive_mode) {
        case DM_COMFORT:    mode = 2; break;
        case DM_ECO_PRO:    mode = 7; break;
        case DM_SPORT:      mode = 4; break;
        case DM_SPORT_PLUS: mode = 5; break;
        default:            mode =s_input.mode == SPORT ? 4 : 2; break;
    }
    if (s_input.light_esc_disabled) {
        mode = 6;
    } else if (s_input.light_tc_disabled) {
        mode = 1;
    }

    uint8_t frame[7] = {0, (uint8_t)(0xF0 | nextCounter(counter)), 0x00, 0x00, mode, 0x11, 0xC0};
    canSendCrc(0x3A7, frame, sizeof(frame), 0x4A);
    return true;
}

// BC button on the turn signal stalk (from CarCluster). Each press steps the line under the
// speedometer.
bool canSendSteeringColumnButtons() {
    // bc_pressed keeps a press shorter than the send interval
    bool pressed = s_input.bc_button || s_input.bc_pressed;
    s_input.bc_pressed = false;

    uint8_t frame[2] = {(uint8_t)(pressed ? 0x40 : 0x00), 0xFF};
    canSend(0x1EE, frame, sizeof(frame));
    return true;
}

// The cluster takes the time as a diagnostic write, UDS 2E D1 13 to address 0x86:
// hour, minute, second, day, month, year (big endian), 00. The request needs two ISO-TP
// frames, the second is sent by canSetClockRest. The answers are not read.
// The cluster has no default time. One is written when its 0x2F8 says it has none, the PC
// time when it arrives and then once a minute.
static uint8_t s_clock_rest[8];
static bool s_clock_rest_pending = false;
static bool s_clock_not_set = false;

void handleClusterClock(const uint8_t* data) {
    if (data[0] == 0xFD) s_clock_not_set = true;
}

bool canSetClock() {
    static uint8_t since_write_s = 255;
    static bool pc_time_written = false;

    if (since_write_s < 255) since_write_s++;

    // 0x2F8 says FD for a moment after a write
    bool write = s_clock_not_set && since_write_s >= 3;
    if (s_input.time_valid && (!pc_time_written || since_write_s >= 60)) write = true;
    if (!write) return false;

    s_clock_not_set = false;
    since_write_s = 0;
    if (s_input.time_valid) pc_time_written = true;

    uint8_t first[8] = {0x86, 0x10, 0x0B, 0x2E, 0xD1, 0x13, s_input.time_hour, s_input.time_minute};
    canSend(0x6F1, first, sizeof(first));

    s_clock_rest[0] = 0x86;
    s_clock_rest[1] = 0x21;
    s_clock_rest[2] = s_input.time_second;
    s_clock_rest[3] = s_input.time_day;
    s_clock_rest[4] = s_input.time_month;
    s_clock_rest[5] = (uint8_t)(s_input.time_year >> 8);
    s_clock_rest[6] = (uint8_t)s_input.time_year;
    s_clock_rest[7] = 0x00;
    s_clock_rest_pending = true;
    return true;
}

bool canSetClockRest() {
    if (!s_clock_rest_pending) return false;
    s_clock_rest_pending = false;
    canSend(0x6F1, s_clock_rest, sizeof(s_clock_rest));
    return true;
}

// Fuel sender resistance in 0.1 ohm, measured against the tank contents the cluster reports
FuelLevelPoint fuelTable[] = {
    {1.000f,   500},
    {0.960f,  1000},
    {0.905f,  1500},
    {0.838f,  2000},
    {0.758f,  2500},
    {0.673f,  3000},
    {0.586f,  3500},
    {0.495f,  4000},
    {0.412f,  4500},
    {0.353f,  5000},
    {0.300f,  5500},
    {0.253f,  6000},
    {0.163f,  7000},
    {0.088f,  8000},
    {0.029f,  9000},
    {0.000f, 10000}
};

uint16_t interpolateFuel(float percent, struct FuelLevelPoint table[], int size) {
    for (int i = 0; i < size - 1; ++i) {
        if (percent >= table[i + 1].fuel_percent) {
            float range = table[i].fuel_percent - table[i + 1].fuel_percent;
            float alpha = (percent - table[i + 1].fuel_percent) / range;
            return (uint16_t)(table[i + 1].meter_level +
                              alpha * ((float)table[i].meter_level - (float)table[i + 1].meter_level) + 0.5f);
        }
    }

    return table[size - 1].meter_level;
}

bool canSendFuel() {
    float fuel = (float)s_input.fuel / 1000.f;
    if (fuel > 1.0f) fuel = 1.0f;
    if (fuel < 0.0f) fuel = 0.0f;

    uint16_t level = interpolateFuel(fuel, fuelTable, sizeof(fuelTable) / sizeof(fuelTable[0]));

    // Left and right tank chamber
    uint8_t frame[5] = {
        (uint8_t)(level & 0xFF), (uint8_t)(level >> 8),
        (uint8_t)(level & 0xFF), (uint8_t)(level >> 8),
        0x00
    };
    canSend(0x349, frame, sizeof(frame));
    return true;
}

static uint16_t elapsedMs(uint32_t& last) {
    uint32_t now = millis();
    uint32_t dt = now - last;
    last = now;
    return dt > 1000 ? 1000 : (uint16_t)dt;
}

// Injected fuel (256 ul per count) and distance (cm) counters, the cluster works out the
// consumption and range from them. Frames from CarCluster, units from
// fatihcesur/bmw-f30-cluster-simhub.
bool canSendFuelConsumption() {
    static uint32_t last_ms = 0;
    static uint32_t remainder = 0;
    static uint32_t total_ul = 0;

    // s_input.fuel_injection is microliters per 100 ms
    remainder += (uint32_t)s_input.fuel_injection * elapsedMs(last_ms);
    total_ul += remainder / 100;
    remainder %= 100;

    uint8_t frame[8] = {0, (uint8_t)(total_ul >> 8), 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1};
    canSendCrc(0x2C4, frame, sizeof(frame), 0xC6);
    return true;
}

bool canSendDistance() {
    static uint8_t counter = 0;
    static uint32_t last_ms = 0;
    static uint32_t remainder = 0;
    static uint16_t distance_cm = 0;

    // s_input.speed is km/h * 10, and 1 km/h is 1 cm per 36 ms
    remainder += (uint32_t)s_input.speed * elapsedMs(last_ms);
    distance_cm += (uint16_t)(remainder / 360);
    remainder %= 360;

    uint8_t frame[5] = {
        0,
        (uint8_t)(0xF0 | nextCounter(counter)),
        (uint8_t)(distance_cm & 0xFF), (uint8_t)(distance_cm >> 8),
        0xF2
    };
    canSendCrc(0x2BB, frame, sizeof(frame), 0xDE);
    return true;
}

// Check control messages
void canSendErrorLight(uint16_t light_id, bool enable) {
    uint8_t frame[] = {
        0x40,
        static_cast<uint8_t>(light_id & 0xFF),
        static_cast<uint8_t>((light_id >> 8) & 0xFF),
        static_cast<uint8_t>(enable ? 0x29 : 0x28),
        0xFF, 0xFF, 0xFF, 0xFF
    };
    canSend(0x5C0, frame, sizeof(frame));
}

bool canSuppressService() {
    canSendErrorLight(SERVICE_LIGHT, false);
    return true;
}

#define DEFINE_CAN_SEND_SYMBOL(fn_name, signal_expr, symbol_const, interval, offset) \
    bool fn_name() {                                                                 \
        static bool last_value = false;                                              \
        static uint8_t counter = offset;                                             \
        bool value = (signal_expr) && (s_input.ignition >= IG_ON);                   \
        counter++;                                                                   \
        if (value != last_value || (counter % (interval) == 0)) {                    \
            last_value = value;                                                      \
            counter = 0;                                                             \
            canSendErrorLight(symbol_const, value);                                  \
            return true;                                                             \
        }                                                                            \
        return false;                                                                \
    }

// Interval = 200 ms
DEFINE_CAN_SEND_SYMBOL(canSendEngineTempYellowSymbol, s_input.engine_temp_yellow && !s_input.engine_temp_red, OVERHEAT_YELLOW, 25, 0)
DEFINE_CAN_SEND_SYMBOL(canSendEngineTempRedSymbol, s_input.engine_temp_red, OVERHEAT_RED, 25, 1)
DEFINE_CAN_SEND_SYMBOL(canSendCheckEngineSymbol, s_input.check_engine, CHECK_ENGINE, 25, 2)
DEFINE_CAN_SEND_SYMBOL(canSendClutchTempSymbol, s_input.clutch_temp, GEARBOX_TEMP_YELLOW, 25, 3)
DEFINE_CAN_SEND_SYMBOL(canSendOilWarningSymbol, s_input.oil_warn, OIL_RED, 25, 4)
DEFINE_CAN_SEND_SYMBOL(canSendBatteryWarningSymbol, s_input.battery_warn , BATTERY_RED, 25, 5)
DEFINE_CAN_SEND_SYMBOL(canSendTireDeflatedFl, s_input.tires.fl_deflated, LOW_TIRE_PRESSURE_FRONT_LEFT, 25, 7)
DEFINE_CAN_SEND_SYMBOL(canSendTireDeflatedFr, s_input.tires.fr_deflated, LOW_TIRE_PRESSURE_FRONT_RIGHT, 25, 8)
DEFINE_CAN_SEND_SYMBOL(canSendTireDeflatedRl, s_input.tires.rl_deflated, LOW_TIRE_PRESSURE_REAR_LEFT, 25, 9)
DEFINE_CAN_SEND_SYMBOL(canSendTireDeflatedRr, s_input.tires.rr_deflated, LOW_TIRE_PRESSURE_REAR_RIGHT, 25, 10)
DEFINE_CAN_SEND_SYMBOL(canSendTireDeflatedAll, s_input.tires.all_deflated, LOW_TIRE_PRESSURE_ALL, 25, 11)
// Stays on until the ignition is switched off
DEFINE_CAN_SEND_SYMBOL(canSendRadiatorSymbol, s_input.radiator_warn, COOLANT_LOW, 25, 12)
DEFINE_CAN_SEND_SYMBOL(canSendDoorOpenFrontLeft, s_input.doors.fl_open, DOOR_OPEN_FRONT_LEFT, 25, 13)
DEFINE_CAN_SEND_SYMBOL(canSendDoorOpenFrontRight, s_input.doors.fr_open, DOOR_OPEN_FRONT_RIGHT, 25, 14)
DEFINE_CAN_SEND_SYMBOL(canSendDoorOpenRearLeft, s_input.doors.rl_open, DOOR_OPEN_REAR_LEFT, 25, 22)
DEFINE_CAN_SEND_SYMBOL(canSendDoorOpenRearRight, s_input.doors.rr_open, DOOR_OPEN_REAR_RIGHT, 25, 23)
DEFINE_CAN_SEND_SYMBOL(canSendTailgateOpen, s_input.doors.tailgate_open, BOOT_OPEN, 25, 15)
DEFINE_CAN_SEND_SYMBOL(canSendBonnetOpen, s_input.doors.bonnet_open, BONNET_OPEN, 25, 6)
DEFINE_CAN_SEND_SYMBOL(canSendBrakeTempSymbol, s_input.brake_temp, HIGH_BRAKE_LOAD, 25, 20)
DEFINE_CAN_SEND_SYMBOL(canSendGearIssueSymbol, s_input.gear_issue, TRANSMISSION_DRIVE_MODERATELY, 25, 21)
// DSC off covers traction control off
DEFINE_CAN_SEND_SYMBOL(canSendEscDisabledSymbol, s_input.light_esc_disabled, DSC_TRIANGLE_DOUBLE, 25, 16)
DEFINE_CAN_SEND_SYMBOL(canSendTcDisabledSymbol, s_input.light_tc_disabled && !s_input.light_esc_disabled, DTC_MODE, 25, 26)
DEFINE_CAN_SEND_SYMBOL(canSendBeaconSymbol, s_input.light_beacon, ALARM_LIGHT, 25, 17)
DEFINE_CAN_SEND_SYMBOL(canSendYellowTriangle, s_input.yellow_triangle, YELLOW_TRIANGLE, 25, 18)
DEFINE_CAN_SEND_SYMBOL(canSendRedTriangle, s_input.red_triangle, RED_TRIANGLE, 25, 19)
DEFINE_CAN_SEND_SYMBOL(canSendCheckeredFlag, s_input.checkered_flag, CHECKERED_FLAG, 25, 23)
DEFINE_CAN_SEND_SYMBOL(canSendLimitYellow, s_input.limit_yellow, LIMIT_YELLOW, 25, 24)
DEFINE_CAN_SEND_SYMBOL(canSendLimitRed, s_input.limit_red, LIMIT_RED, 25, 25)

// Interval = 50 ms
// One DSC lamp for both, it flashes as the game pulses the bits
DEFINE_CAN_SEND_SYMBOL(canSendEscSymbol, s_input.light_esc_active || s_input.light_tc_active, DSC_TRIANGLE_SYMBOL_ONLY, 100, 2)

bool canSendCustomSymbol() {
    static bool last_state = false;
    static uint16_t last_number = 0;
    static uint8_t counter = 0;
    const bool changed = s_input.custom_light != last_number || last_state != s_input.custom_light_on;

    if (changed || (++counter % 10 == 0)) {
        last_state = s_input.custom_light_on;
        last_number = s_input.custom_light;
        counter = 0;
        canSendErrorLight(last_number, last_state);
        return true;
    }
    return false;
}

#ifdef READ_FRAMES_FROM_CLUSTER_2CA
void handle2CA(const uint8_t* data) {
    float temp_c = (data[0] / 2.0f) - 40.0f;
    serial_printf(pc, "[CAN2CA] Outside Temp: %.1f°C\n", temp_c);
}
#endif

static const CanHandlerEntry handlers[] = {
    { 0x2F8, handleClusterClock },
#ifdef READ_FRAMES_FROM_CLUSTER_2CA
    { 0x2CA, handle2CA },
#endif
};
static const size_t handler_count = sizeof(handlers) / sizeof(handlers[0]);

const int MaxQueueSize = 64;
bool (*canQueue[MaxQueueSize])();
int queueHead = 0;
int queueTail = 0;

bool queueIsEmpty() { return queueHead == queueTail; }
bool queueIsFull() { return ((queueTail + 1) % MaxQueueSize) == queueHead; }
void queuePush(bool (*f)()) { if (!queueIsFull()) { canQueue[queueTail] = f; queueTail = (queueTail + 1) % MaxQueueSize; } }
bool (*queuePop())() { if (!queueIsEmpty()) { bool (*f)() = canQueue[queueHead]; queueHead = (queueHead + 1) % MaxQueueSize; return f; } return nullptr; }

void setup() {
#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    serialBegin();

    canBegin();

#if defined(USE_AD5272_AMBIENT)
    if (!ambientTemp.begin()) {
        pc.println("AD5272 ambient temp sensor init failed");
    } else {
        pc.println("AD5272 initialized successfully");
        ambientTemp.setTemperature((float)s_input.ambient_temp / 10.f);
    }
#endif
}

#if defined(USE_AD5272_AMBIENT)
void updateAmbientTemperature() {
    static float current_temp = (float)s_input.ambient_temp / 10.f;
    float target_temp = (float)s_input.ambient_temp / 10.f;

    // At most 0.1 °C per second
    const float max_change = 0.1f;
    float delta = target_temp - current_temp;

    if (delta > max_change) {
        current_temp += max_change;
    } else if (delta < -max_change) {
        current_temp -= max_change;
    } else {
        current_temp = target_temp;
    }

    ambientTemp.setTemperature(current_temp);
}
#endif

void loop() {
    uint32_t now_us = micros();
    uint32_t now_ms = now_us / 1000;

    // Every 10 ms
    if (now_ms - s_timers.lastTime >= 10) {
        s_timers.lastTime = now_ms;

        // 20 ms
        if (s_timers.canCounter % 2 == 0) {
            queuePush(canSendRPM);
            queuePush(canSendSpeed);
            queuePush(canSendAcceleratorPedal);
        }
        // 50 ms
        if (s_timers.canCounter % 5 == 1) {
            queuePush(canSendEscSymbol);
            queuePush(canSendLights);
        }
        // 100 ms group 1
        if (s_timers.canCounter % 10 == 3) {
            queuePush(canSendIgnitionFrame);
            queuePush(canSendGearboxData);
            queuePush(canSendEngineTemperatures);
            queuePush(canSendIndicator);
            queuePush(canSendHandbrake);
            queuePush(canSendFuel);
            queuePush(canSendFuelConsumption);
            queuePush(canSendDistance);
            queuePush(canSendAbs);
            queuePush(canSendPowerSteering);
            queuePush(canSendCruiseControl);
            queuePush(canSendAirbag);
        }
        // 100 ms group 2
        if (s_timers.canCounter % 10 == 7) {
            queuePush(canSendSeatbelt);
            queuePush(canSendTirePressure);
            queuePush(canSendAcc);
            queuePush(canSendSafetyCounter);
            queuePush(canSendSteeringColumnButtons);
            queuePush(canSendGatewayAlive);
            queuePush(canSendEngineTorque);
            queuePush(canSendCentralLocking);
            queuePush(canSendDscStatus);
            queuePush(canSendDrivingLightsStatus);
            queuePush(canSendSpeedConversionFactor);
            queuePush(canSendDrivingDynamicsSwitchConfig);
        }
        // 200 ms group 1
        if (s_timers.canCounter % 20 == 9) {
            queuePush(canSendEngineTempYellowSymbol);
            queuePush(canSendEngineTempRedSymbol);
            queuePush(canSendCheckEngineSymbol);
            queuePush(canSendClutchTempSymbol);
            queuePush(canSendOilWarningSymbol);
            queuePush(canSendBatteryWarningSymbol);
            queuePush(canSendCustomSymbol);
            queuePush(canSendTireDeflatedFl);
            queuePush(canSendTireDeflatedFr);
            queuePush(canSendTireDeflatedRl);
            queuePush(canSendTireDeflatedRr);
            queuePush(canSendTireDeflatedAll);
            queuePush(canSendRadiatorSymbol);
            queuePush(canSendLevelControl);
            queuePush(canSendEmergencyCall);
        }
        // 200 ms group 2
        if (s_timers.canCounter % 20 == 19) {
            queuePush(canSendDoorOpenFrontLeft);
            queuePush(canSendDoorOpenFrontRight);
            queuePush(canSendDoorOpenRearLeft);
            queuePush(canSendDoorOpenRearRight);
            queuePush(canSendTailgateOpen);
            queuePush(canSendBonnetOpen);
            queuePush(canSendBrakeTempSymbol);
            queuePush(canSendGearIssueSymbol);
            queuePush(canSendEscDisabledSymbol);
            queuePush(canSendTcDisabledSymbol);
            queuePush(canSendBeaconSymbol);
            queuePush(canSendYellowTriangle);
            queuePush(canSendRedTriangle);
            queuePush(canSendCheckeredFlag);
            queuePush(canSendLimitYellow);
            queuePush(canSendLimitRed);
            queuePush(canSendHighBeamAssist);
            queuePush(canSendAmbientBrightness);
        }
        // 500 ms
        if (s_timers.canCounter % 50 == 5) {
            queuePush(canSuppressService);
            queuePush(canSendNetworkManagement);
        }
        // 1 s
        if (s_timers.canCounter % 100 == 35) {
            queuePush(canSetClock);
            queuePush(canSendBacklight);
            queuePush(canSendDriveMode);
            queuePush(canSendVehicleState);
#if defined(USE_AD5272_AMBIENT)
            updateAmbientTemperature();
#endif
        }
        // Second frame of the clock request, 30 ms after the first
        if (s_timers.canCounter % 100 == 38) {
            queuePush(canSetClockRest);
        }
        s_timers.canCounter++;
    }

    // Space out the frames, see CAN_FRAME_INTERVAL_US in config.h. Tasks that sent nothing
    // do not hold up the next one.
    if (now_us - s_timers.lastTaskTime >= CAN_FRAME_INTERVAL_US) {
        bool (*task)() = queuePop();
        if (task && task()) {
            s_timers.lastTaskTime = now_us;
        }
    }

    serialPoll();

    canPoll(handlers, handler_count);
}
