#pragma once

// CAN adapter: pick one option below.
// The cluster's CAN bus runs at 500 kbit/s.

// MCP2515 SPI adapter. Install "mcp_can" library.
// More at https://github.com/coryjfowler/MCP_CAN_lib
//#define USE_MCP_CAN_SPI

#ifndef MCP_CAN_SPI_SPEED
    // Set to MCP_16MHZ if you have an adapter with 16MHz crystal
    #define MCP_CAN_SPI_SPEED MCP_8MHZ
#endif

#ifndef MCP_CAN_SPI_CS_PIN
    #define MCP_CAN_SPI_CS_PIN 10
#endif

// ESP32 built-in TWAI controller with an SN65HVD230 (or compatible) transceiver.
// No external library required; uses the TWAI driver from the ESP32 Arduino core.
//#define USE_ESP32_TWAI

#ifndef TWAI_TX_PIN
    #define TWAI_TX_PIN 23
#endif

#ifndef TWAI_RX_PIN
    #define TWAI_RX_PIN 22
#endif

#if defined(USE_MCP_CAN_SPI) == defined(USE_ESP32_TWAI)
    #error "Select exactly one CAN adapter in config.h"
#endif

#ifndef CAN_FRAME_INTERVAL_US
    // Minimum time between sent CAN frames
    #define CAN_FRAME_INTERVAL_US 1000
#endif

// Serial baud rate
#ifndef PC_SERIAL_BAUD
    #define PC_SERIAL_BAUD 921600
#endif

// Cluster parameters
#ifndef NUMBER_OF_GEARS
    #define NUMBER_OF_GEARS 8
#endif

// AD5272 digital potentiometer for ambient temperature
//#define USE_AD5272_AMBIENT

#ifndef AD5272_I2C_ADDRESS
    #define AD5272_I2C_ADDRESS 0x2E
#endif

#ifndef AD5272_I2C_SDA_PIN
    #define AD5272_I2C_SDA_PIN 18
#endif

#ifndef AD5272_I2C_SCL_PIN
    #define AD5272_I2C_SCL_PIN 19
#endif

// Debug: uncomment to log CAN frames from cluster
//#define READ_FRAMES_FROM_CLUSTER_2CA  // Outside temperature
