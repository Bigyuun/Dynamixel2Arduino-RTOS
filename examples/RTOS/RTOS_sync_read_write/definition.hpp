#pragma once

#ifndef DEFINITION_HPP_
#define DEFINITION_HPP_

/*******************************************************************************
* Copyright 2022 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

// Tutorial Video: https://youtu.be/msWlMyx8Nrw
// Example Environment
//
// - DYNAMIXEL: X series
//              ID = 1, Baudrate = 57600bps, DYNAMIXEL Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/
// - Adjust the position_p_gain, position_i_gain, position_d_gain values
// Author: David Park

#include <Dynamixel2Arduino.h>
// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

//This namespace is required to use Control table item names
using namespace ControlTableItem;

//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//
// Project is a simple example of how to get FreeRtos running on a SamD21 processor
// Project can be used as a template to build your projects off of as well
//**************************************************************************
#include <FreeRTOS_SAMD21.h>
#define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
//#define  ERROR_LED_PIN  2 //Led Pin: samd21 xplained board
#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high
// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
#define SERIAL          SerialUSB //Sparkfun Samd21 Boards
//#define SERIAL          Serial //Adafruit, other Samd21 Boards


/**************************************************************************
* User Libraries
***************************************************************************/
#include <vector>
#include <algorithm>
#include "control_table.hpp"

/** custom **/
#define SYNC_MODE true  // if not use, modify to false

#define MONITORING_MODE true
#define DXL_ID_CNT 2
#define DXL_BAUDRATE  57600 // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
#define BROADCAST_ID  254
#define DYNAMIXEL_PROTOCOL_VERSION 2.0  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
// #define OP_MODE   OP_EXTENDED_POSITION
#define OP_MODE   OP_CURRENT_BASED_POSITION

#define DXL_POSITION_P_GAIN 3000
#define DXL_POSITION_I_GAIN 0
#define DXL_POSITION_D_GAIN 3000

#define DXL_CURRENT_LIMIT 200
// #define VELOCITY_P_GAIN 
// #define VELOCITY_I_GAIN

#define SERIAL_BAUDRATE 115200
#define SERIAL_SETTIMEOUT 10
#define SERIAL_WRITE_FREQUENCY  100   // Hz

#define HOMING_DIRECTION 1
#define HOMING_VELOCITY  100 * HOMING_DIRECTION
#define HOMING_CURRENT  100 * HOMING_DIRECTION


enum OpMode{
  kStop = 0,
  kEnable = 1,
  kHoming = 5,
};

/**
* @author DY
* @brief
    If you want to get differ data(PWM, etc ...), put some member variables adjust data type of 'Control Table'
    - https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#control-table-data-address
*/
typedef struct SyncReadData {
  int8_t operation_mode;
  int8_t torque_enable;
  int8_t moving;
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
  int8_t hardware_error;
} __attribute__((packed)) SyncReadData_t;

typedef struct SyncReadTorqueEnableData{
  int8_t torque_enable;
} __attribute__((packed)) SyncReadTorqueEnableData_t;

typedef struct SyncReadMovingData{
  int8_t moving;
} __attribute__((packed)) SyncReadMovingData_t;

typedef struct SyncReadPresentCurrentData{
  int16_t present_current;
} __attribute__((packed)) SyncReadPresentCurrentData_t;

typedef struct SyncReadPresentVelocityData{
  int32_t present_velocity;
} __attribute__((packed)) SyncReadPresentVelocityData_t;

typedef struct SyncReadPresentPositionData{
  int32_t present_position;
} __attribute__((packed)) SyncReadPresentPositionData_t;

typedef struct SyncReadHardwareErrorData{
  int8_t hardware_error;
} __attribute__((packed)) SyncReadErrorCodenData_t;

////////////////////////////////////////////////////////////////////
typedef struct SyncWriteData {
  int16_t goal_current;
  int32_t goal_velocity;
  int32_t goal_position;
} __attribute__((packed)) SyncWriteData_t;

// TEST
typedef struct sr_data{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

// typedef struct sw_data{
//   int32_t goal_velocity;
// } __attribute__((packed)) sw_data_t;













#endif /* DEFINITION_HPP_ */
