#pragma once

enum INDICATOR {
    I_OFF = 0,
    I_LEFT,
    I_RIGHT,
    I_HAZZARD
};

// 0x21A byte 0
enum CAN_LIGHTS {
    L_HIGH_BEAM = 0x02,
    L_LOW_BEAM = 0x04,  // Main lights
    L_FRONT_FOG = 0x20,
    L_REAR_FOG = 0x40
};

enum GEAR {
    PARK = -1,
    REVERSE = 0,
    NEUTRAL = 1,
    DRIVE = 2
};

enum GEAR_MANUAL {
    NONE = 0, M1, M2, M3, M4, M5, M6, M7, M8
};

enum GEAR_MODE {
    NORMAL,
    SPORT
};

enum IGNITION_STATE {
    IG_OFF = 0,
    IG_ACCESSORY = 1,
    IG_ON = 2,
    IG_STARTER = 3
};

enum ErrorLightID : uint16_t {
    BOOT_OPEN = 19,
    FIX_CAR = 21,
    YELLOW_WARNING = 24,
    OIL_LEVEL_LOW = 28,
    CHECK_ENGINE_DOUBLE = 31,
    CHECK_ENGINE = 34,
    DTC_TRIANGLE_DOUBLE_EXCLAMATION_MARK = 35,
    YELLOW_TRIANGLE = 37,
    OVERHEAT_RED = 39,
    START_FOOT_BRAKE = 40,
    RED_TRIANGLE = 41,
    ABS_DTC_TRIANGLE_WARNING = 42,
    SUSPENSION_WARNING = 43,
    TIRE_PRESSURE_YELLOW = 50,
    LIMIT_RED = 62,
    TIRE_PRESSURE_RED = 63,
    CRUISE_WARNING = 69,
    LIMIT_YELLOW = 78,
    COLD_WEATHER = 79,
    BATTERY_YELLOW = 161,
    DTC_MODE = 184,
    LOW_TIRE_PRESSURE_FRONT_LEFT = 139,
    LOW_TIRE_PRESSURE_REAR_RIGHT = 140,
    LOW_TIRE_PRESSURE_REAR_LEFT = 141,
    LOW_TIRE_PRESSURE_FRONT_RIGHT = 143,
    LOW_TIRE_PRESSURE_ALL = 147,
    DSC_TRIANGLE_DOUBLE = 36,
    DSC_TRIANGLE_SYMBOL_ONLY = 215,
    FOOT_TO_BRAKE = 244,
    GEARBOX_ISSUE_YELLOW = 288,
    OIL_RED = 212,
    BATTERY_RED = 213,
    SOS_ERROR = 299,
    OVERHEAT_YELLOW = 257,
    FUEL_WARNING = 275,
    SERVICE_LIGHT = 281,
    GEARBOX_TEMP_YELLOW = 104,
    GEARBOX_TEMP_RED = 105,
    BRAKES_HOT = 353,
    COOLANT_LOW = 166,
    DOOR_OPEN_FRONT_RIGHT = 14,
    DOOR_OPEN_FRONT_LEFT = 15,
    DOOR_OPEN_REAR_LEFT = 16,
    DOOR_OPEN_REAR_RIGHT = 17,
    ALARM_LIGHT_EXCLAMATION = 162,
    ALARM_LIGHT = 508,
    BRAKE_SYMBOL_RED = 308,
    CHECKERED_FLAG = 446
};

struct FuelLevelPoint {
    float fuel_percent;  // 1.0 = 100%, 0.75 = 75%, etc.
    uint16_t meter_level; // Value to send to the cluster
};

struct STimers {
    uint32_t lastTime = 0;
    uint32_t lastTaskTime = 0;
    uint16_t canCounter = 0;
};

struct SInput {
    IGNITION_STATE ignition = IG_ON;
    INDICATOR indicator_state = I_OFF;

    bool time_valid = false; // Set when the PC has sent a time
    uint16_t time_year = 2010;
    uint8_t time_month = 1;
    uint8_t time_day = 1;
    uint8_t time_hour = 12;
    uint8_t time_minute = 0;
    uint8_t time_second = 0;

    uint16_t rpm = 0;
    uint16_t speed = 0;
    GEAR currentGear = PARK;
    GEAR_MANUAL explicitGear = NONE;
    GEAR_MODE mode = NORMAL;
    uint16_t fuel = 1000;
    uint16_t fuel_injection = 0;
    uint8_t water_temp = 0;
    uint8_t oil_temp = 0;
    uint16_t custom_light = 0;

    bool custom_light_on = false;
    bool light_shift = false;
    bool light_highbeam = false;
    bool light_lowbeam = true;
    bool light_fog = false;
    bool light_tc_active = false;
    bool light_esc_active = false;
    bool light_tc_disabled = false;
    bool light_esc_disabled = false;
    bool light_beacon = false;
    bool oil_warn = false;
    bool battery_warn = false;
    bool abs_warn = false;
    bool engine_temp_yellow = false;
    bool engine_temp_red = false;
    bool check_engine = false;
    bool clutch_temp = false;
    bool brake_temp = false;
    bool radiator_warn = false;
    bool engine_running = true;
    int16_t ambient_temp = 200;
    bool yellow_triangle = false;
    bool red_triangle = false;
    bool gear_issue = false;
    bool checkered_flag = false;
    bool limit_yellow = false;
    bool limit_red = false;

    struct {
        bool fl_deflated = false;
        bool fr_deflated = false;
        bool rl_deflated = false;
        bool rr_deflated = false;
        bool all_deflated = false;
    } tires;

    struct {
        bool fl_open = false;
        bool fr_open = false;
        bool rl_open = false;
        bool rr_open = false;
        bool tailgate_open = false;
    } doors;

    bool handbrake = false;

    // bc_pressed stays set until the press has been sent
    bool bc_button = false;
    bool bc_pressed = false;

    bool coasting = false;

    // 0 unknown, 1 comfort, 2 sport, 3 sport+
    uint8_t drive_mode = 0;

    struct {
        bool enabled = false;
        uint16_t speed = 0;

        struct {
            uint8_t distance = 0; // 0: no ACC, 1-4: distances from short to far
            bool yellow_car_static = false;
            bool foot_on_brake = false;
            bool red_car_blinking = false;
        } acc;
    } cruise;
};
