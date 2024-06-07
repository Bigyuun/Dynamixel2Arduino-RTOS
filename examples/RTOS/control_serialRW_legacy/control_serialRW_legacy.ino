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
  #define SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

using namespace ControlTableItem;

// struct DynamixelMotor
// {
//   uint8_t id = 1;
//   uint32_t baudrate = 57600;
//   float dxl_protocol_version = 2.0;
//   bool torque_enable = 0;
//   uint8_t op_mode;
//   float goal_position = 0;
// };


const uint8_t DXL_ID = 1;
const uint32_t DXL_BAUDRATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

int32_t goal_position[2] = {1000, 1500};
int8_t direction = 0;
unsigned long timer = 0;

// Position PID Gains
// Adjust these gains to tune the behavior of DYNAMIXEL
uint16_t position_p_gain = 3000;
uint16_t position_i_gain = 0;
uint16_t position_d_gain = 3000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;


//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//
// Project is a simple example of how to get FreeRtos running on a SamD21 processor
// Project can be used as a template to build your projects off of as well
//
//**************************************************************************

#include <FreeRTOS_SAMD21.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
//#define  ERROR_LED_PIN  2 //Led Pin: samd21 xplained board

#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high

// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
#define SERIAL          SerialUSB //Sparkfun Samd21 Boards
//#define SERIAL          Serial //Adafruit, other Samd21 Boards

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_serialreadTask;
TaskHandle_t Handle_serialwriteTask;
TaskHandle_t Handle_DynamixelTask;
// synchronizing
SemaphoreHandle_t serial_mutex;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account processor speed
// Use these instead of delay(...) in rtos tasks
//**************************************************************************
void DelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void DelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void DelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

static void thread_Dynamixel_node( void *pvParameters ) 
{
  SERIAL.println("Dynamixel node is up.");
  while(true){
    /** 
      * write the code
      */
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    SERIAL.print("[Thread-Dynamixel State] Present Current(mA)= ");
    SERIAL.println(dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE));
    SERIAL.flush();
    xSemaphoreGive(serial_mutex);
    vTaskDelay( 50 / portTICK_PERIOD_MS );
  }
}

/**
  * @brief Serial read thread for implementing the command based on protocol.
  */
static void thread_serial_read( void *pvParameters ) 
{
  SERIAL.println("Serial Read node is up.");
  while(true){
    if (SERIAL.available() <= 0) {
      continue;
    }
    // received messages update
    String msg = SERIAL.readString();
    char command[50] = "";
    bool suc = parsing_serial_command(msg, command);
    dxl.setGoalPosition(DXL_ID, atof(command));
    
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    SERIAL.print("[Thread-Serial READ] Target Position= ");
    SERIAL.println(String(command));
    SERIAL.flush();
    xSemaphoreGive(serial_mutex);

    vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
}


/**
  * @brief Serial write thread for echo of motor's states
  */
static void thread_serial_write( void *pvParameters ) 
{
  SERIAL.println("Serial write node is up.");
  while(1){
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    SERIAL.print("[Thread-Serial Write] Present Position(inc)= ");
    SERIAL.println(dxl.getPresentPosition(DXL_ID));
    SERIAL.print("CURRENT(mA) = ");
    SERIAL.println(dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE));
    SERIAL.flush();
    xSemaphoreGive(serial_mutex);

    vTaskDelay( 10 / portTICK_PERIOD_MS );
  }
}

// void call_dynamixel_function(String command)
// {
//   switch(command) {
//     case: 'OP'
//   }
// }

void serial_init()
{
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  SERIAL.setTimeout(100);
  SERIAL.begin(115200);
  // SERIAL.begin(57600);
  while(!SERIAL);
  SERIAL.print("SERIAL: ");
  SERIAL.println(SERIAL);

  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  serial_mutex = xSemaphoreCreateMutex();
  if (serial_mutex == NULL){
    SERIAL.println("Mutex can not be created.");
  }
  else{
    SERIAL.println("Mutex is created completely.");
  }
}

void dynamixel_init()
{
  bool success;
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BAUDRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  success = dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  SERIAL.println(success);
  // Get DYNAMIXEL information
  success = dxl.ping(DXL_ID);
  SERIAL.print("ping(): ");
  SERIAL.println(success);

  // Turn off torque when configuring items in EEPROM area
  success = dxl.torqueOff(DXL_ID);
  SERIAL.print("torqueOff(): ");
  SERIAL.println(success);
  success = dxl.setOperatingMode(DXL_ID, OP_POSITION);
  SERIAL.print("setOperatingMode(): ");
  SERIAL.println(success);
  success = dxl.torqueOn(DXL_ID);
  SERIAL.print("torqueOn(): ");
  SERIAL.println(success);
  
  // Set Position PID Gains
  success = dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, position_p_gain);
  SERIAL.print("writeControlTableItem(): ");
  SERIAL.println(success);
  success = dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, position_i_gain);
  SERIAL.print("writeControlTableItem(): ");
  SERIAL.println(success);
  success = dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, position_d_gain);
  SERIAL.print("writeControlTableItem(): ");
  SERIAL.println(success);
  SERIAL.println("Dynamixel initilaizing... OK");
}

bool parsing_serial_command(String msg, char * command)
{
  // find '/'
  int ipos0 = msg.indexOf('/');
  if (ipos0 < 0)
    // SERIAL.println("ipos'/'<0. fail");
    return false;
  // find ';'
  int ipos1 = msg.indexOf(';');
  if (ipos1 < 0)
    // SERIAL.println("ipos';'<0. fail");
    return false;
  // get valid message
  String valid_msg = msg.substring(ipos0 + 1, ipos1);
  valid_msg.toCharArray(command, valid_msg.length()+1);

  return true;
}

void print_on_mutex(SemaphoreHandle_t serial_mutex, String msg)
{
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  SERIAL.println(msg);
  SERIAL.flush();
  xSemaphoreGive(serial_mutex);
}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
void print_task_info()
{
  int x;
  int measurement;
  static char ptrTaskList[400]; //temporary string buffer for task stats
  
  SERIAL.println("Task Monitor: Started");

  // myDelayMs(10000); // print every 10 seconds
  SERIAL.flush();
  SERIAL.println("");			 
  SERIAL.println("****************************************************");
  SERIAL.print("Free Heap: ");
  SERIAL.print(xPortGetFreeHeapSize());
  SERIAL.println(" bytes");

  SERIAL.print("Min Heap: ");
  SERIAL.print(xPortGetMinimumEverFreeHeapSize());
  SERIAL.println(" bytes");
  SERIAL.flush();

  SERIAL.println("****************************************************");
  SERIAL.println("Task            ABS             %Util");
  SERIAL.println("****************************************************");

  vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
  SERIAL.println(ptrTaskList); //prints out already formatted stats
  SERIAL.flush();

  SERIAL.println("****************************************************");
  SERIAL.println("Task            State   Prio    Stack   Num     Core" );
  SERIAL.println("****************************************************");

  vTaskList(ptrTaskList); //save stats to char array
  SERIAL.println(ptrTaskList); //prints out already formatted stats
  SERIAL.flush();

  SERIAL.println("****************************************************");
  SERIAL.println("[Stacks Free Bytes Remaining] ");

  measurement = uxTaskGetStackHighWaterMark( Handle_DynamixelTask );
  SERIAL.print("Dynamixel Task: ");
  SERIAL.println(measurement);

  measurement = uxTaskGetStackHighWaterMark( Handle_serialwriteTask );
  SERIAL.print("Serial Write Task: ");
  SERIAL.println(measurement);

  measurement = uxTaskGetStackHighWaterMark( Handle_serialreadTask );
  SERIAL.print("Serial Read Task: ");
  SERIAL.println(measurement);
  
  SERIAL.println("****************************************************");
  SERIAL.flush();
}

//*****************************************************************
void setup() 
{
  serial_init();
  dynamixel_init();
  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks

  xTaskCreate(thread_serial_write,
    "Task Serial Write",
    256,
    NULL,
    tskIDLE_PRIORITY + 2,
    &Handle_serialwriteTask);
  
  xTaskCreate(thread_serial_read,
    "Task Serial Read",
    256,
    NULL,
    tskIDLE_PRIORITY + 1,
    &Handle_serialreadTask);

  // xTaskCreate(thread_Dynamixel_node,
  //   "Task Dynamixel",
  //   256,
  //   NULL,
  //   tskIDLE_PRIORITY + 1,
  //   &Handle_DynamixelTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
  // error scheduler failed to start
  // should never get here
  while(1)
  {
    SERIAL.println("Scheduler Failed! \n");
    SERIAL.flush();
    delay(1000);
  }
  
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    // Optional commands, can comment/uncomment below
    // SERIAL.print("HHHHHHHHHHHHHHHHHHHHHHHHH"); //print out dots in terminal, we only do this when the RTOS is in the idle state
    // SERIAL.flush();
    // delay(100); //delay is interrupt friendly, unlike vNopDelayMS
}
//*****************************************************************

