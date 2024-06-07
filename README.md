# DYNAMIXEL2Arduino [![Build Status](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/workflows/dynamixel2arduino_ci/badge.svg)](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino)


## Summary

This is the Dynamixel motor control code with Dynamixel2Arduino API on RTOS system.

It consist of 2 thread, serial-write & serial-read.

We can publish the command with arguments as string, and subscribe the motor states from OpenRB-150.

## Environments

- MCU : OpenRB-150
- Arduino IDE (2.3.2)

## Protocol of serial-communication

- STX → /
- CTX → ;
- Substances
    - Read : Function name and its arguments
    - Write : motor state

## Code
👉🏻 [Demo code] (https://github.com/Bigyuun/Dynamixel2Arduino-RTOS/blob/master/examples/RTOS/control_serialRW/control_serialRW.ino)

### configuration
- control_serialRW.ino
- definition.hpp


## Example
### Serial Write (MCU to PC)

- **Form**  
  /motor_id, errcode, MOVING, PresentPosition(degree), PresentVelocity, PresentCurrent;
  
  → e.g. /1,0,0,87.912003,0.000000,0.000000;
    
    ![Arduino Serial Monitor](./images/Untitled.png)
    

### Serial Read (PC to MCU)

- **Form**
  
  /Functinos name(Dynamixel2Arduino), arg1, arg2, ...;

- **Dynamixel2Arduino API**
    - For using ‘dxl.setOperatingMode(1, OP_POSITION)’
    → /setOperatingMode,1,3;
    
      ![Arduino Serial Monitor](./images/Untitled0.png)

- **Other custom function**
    - Monitoring as protocol
      
      If you want to see without protocol, you can send 
      
      → /monitorOn;
        
        ![Arduino Serial Monitor](./images/Untitled.png)
        
    - Monitoring as User
    
      Else if you want to see the motor states with protocol, you can send
      
      → /monitorOff;
        
        ![Arduino Serial Monitor](./images/Untitled1.png)
        
    - Task information
      
      If you want to check the task information about each multi-thread each, you can send
      
      → /task;
        
        ![Arduino Serial Monitor](./images/Untitled2.png)

## Serial and Direction Pin definitions of ROBOTIS controllers
 - When running DYNAMIXEL without DYNAMIXEL Shields on OpenCM9.04, OpenCR or custom boards, you might need to change the Serial and DYNAMIXEL Direction Pin.
 - We provide the information below to make it easier to define Serial and Direction pins for specific hardware.

    |Board Name|Serial|Direction Pin|Note|
    |:-:|:-:|:-:|:-:|
    |OpenCM9.04|Serial1|28|Uploading Arduino Sketch will erase the factory firmware. The firmware can be recovered with DYNAMIXEL Wizard 2.0|
    |OpenCM485EXP|Serial3|22||
    |OpenCR|Serial3|84|OpenCR has an FET switch to control power supply to DYNAMIXEL. ([Reference link](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78))|
    |OpenRB-150|Serial1|(-1)Automatic|-|


## How to add new DYNAMIXEL model.
 - For the convenience of the user, Dynamixel2Arduino API hardcodes some information in the control table and stores it in flash.
 - To do this, you need to add code to some files. In this regard, please refer to [PR#3](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/pull/3) and [PR#7](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/pull/7)

## How to create custom PortHandler Class
 - Please refer to [port_handler.h](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/blob/master/src/utility/port_handler.h)
 - Create a new class by inheriting PortHandler as public. (Like SerialPortHandler and USBSerialPortHandler)
