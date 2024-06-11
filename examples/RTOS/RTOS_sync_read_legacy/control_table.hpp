#ifndef CONTROL_TABLE_HPP_
#define CONTROL_TABLE_HPP_


/**
* @author DY
* @note Control Table parameters
    Address ID
    Address length
*/
///////////////////////////////////////////////////////////////////////////////////////
// Address ID
///////////////////////////////////////////////////////////////////////////////////////

// EEPROM area
#define ADDR_MODEL_UMBER            0
#define ADDR_MODEL_INFORMATION      2
#define ADDR_FIRMWARE_VERSION       6
#define ADDR_ID                     7
#define ADDR_BAUD_RATE              8
#define ADDR_RETURN_DELAY           9
#define ADDR_DRIVE_MODE             10
#define ADDR_OPERATING_MODE         11
#define ADDR_SECONDARY_ID           12
#define ADDR_PROTOCOL_TYPE          13
#define ADDR_HOMING_OFFSET          20
#define ADDR_MOVING_THRESHOLD       24
#define ADDR_TEMPERATURE_LIMIT      31
#define ADDR_MAX_VOLTAGE            32
#define ADDR_MIN_VOLTAGE            34
#define ADDR_PWM_LIMIT              36
#define ADDR_CURRENT_LIMIT          38
#define ADDR_VELOCITY_LIMIT         44
#define ADDR_MAX_POSITION           48
#define ADDR_MIN_POSITION           52
#define ADDR_STARTUP_CONFIGURATION  60
#define ADDR_PWM_SLOPE              62
#define ADDR_SHUTDOWN               63

// RAM area
#define ADDR_TORQUE_ENABLE          64
#define ADDR_LED                    65
#define ADDR_STATUS_RETURN          68
#define ADDR_REGISTERED_INSTRUCTION 69
#define ADDR_HARDWARE_ERROR         70
#define ADDR_VELOCITY_I             76
#define ADDR_VELOCITY_P             78
#define ADDR_POSITION_D             80
#define ADDR_POSITION_I             82
#define ADDR_POSITION_P             84
#define ADDR_FEEDFORWARD_2ND        88
#define ADDR_FEEDFORWARD_1ST        90
#define ADDR_BUS_WATCHDOG           98
#define ADDR_GOAL_PWM               100
#define ADDR_GOAL_CURRENT           102
#define ADDR_GOAL_VELOCITY          104
#define ADDR_PROFILE_ACCELERATION   108
#define ADDR_PROFILE_VELOCITY       112
#define ADDR_GOAL_POSITION          116
#define ADDR_REALTIME_TICK          120
#define ADDR_MOVING               122
#define ADDR_MOVING_STATUS          123
#define ADDR_PRESENT_PWM            124
#define ADDR_PRESENT_CURRENT        126
#define ADDR_PRESENT_VELOCITY       128
#define ADDR_PRESENT_POSITION       132
#define ADDR_VELOCITY_TRAJECTORY    136
#define ADDR_POSITION_TRAJECTORY    140
#define ADDR_PRESENT_INPUT          144
#define ADDR_PRESENT_TEMPERATURE    146


///////////////////////////////////////////////////////////////////////////////////////
// Address length
///////////////////////////////////////////////////////////////////////////////////////

// EEPROM area
#define ADDR_LEN_MODEL_UMBER            2
#define ADDR_LEN_MODEL_INFORMATION      4
#define ADDR_LEN_FIRMWARE_VERSION       1
#define ADDR_LEN_ID                     1
#define ADDR_LEN_BAUD_RATE              1
#define ADDR_LEN_RETURN_DELAY           1
#define ADDR_LEN_DRIVE_MODE             1
#define ADDR_LEN_OPERATING_MODE         1
#define ADDR_LEN_SECONDARY_ID           1
#define ADDR_LEN_PROTOCOL_TYPE          1
#define ADDR_LEN_HOMING_OFFSET          4
#define ADDR_LEN_MOVING_THRESHOLD       4
#define ADDR_LEN_TEMPERATURE_LIMIT      1
#define ADDR_LEN_MAX_VOLTAGE            2
#define ADDR_LEN_MIN_VOLTAGE            2
#define ADDR_LEN_PWM_LIMIT              2
#define ADDR_LEN_CURRENT_LIMIT          2
#define ADDR_LEN_VELOCITY_LIMIT         4
#define ADDR_LEN_MAX_POSITION           4
#define ADDR_LEN_MIN_POSITION           4
#define ADDR_LEN_STARTUP_CONFIGURATION  1
#define ADDR_LEN_PWM_SLOPE              1
#define ADDR_LEN_SHUTDOWN               1

// RAM area
#define ADDR_LEN_TORQUE_ENABLE          1
#define ADDR_LEN_LED                    1
#define ADDR_LEN_STATUS_RETURN          1
#define ADDR_LEN_REGISTERED_INSTRUCTION 1
#define ADDR_LEN_HARDWARE_ERROR         1
#define ADDR_LEN_VELOCITY_I             2
#define ADDR_LEN_VELOCITY_P             2
#define ADDR_LEN_POSITION_D             2
#define ADDR_LEN_POSITION_I             2
#define ADDR_LEN_POSITION_P             2
#define ADDR_LEN_FEEDFORWARD_2ND        2
#define ADDR_LEN_FEEDFORWARD_1ST        2
#define ADDR_LEN_BUS_WATCHDOG           1
#define ADDR_LEN_GOAL_PWM               2
#define ADDR_LEN_GOAL_CURRENT           2
#define ADDR_LEN_GOAL_VELOCITY          4
#define ADDR_LEN_PROFILE_ACCELERATION   4
#define ADDR_LEN_PROFILE_VELOCITY       4
#define ADDR_LEN_GOAL_POSITION          4
#define ADDR_LEN_REALTIME_TICK          2
#define ADDR_LEN_MOVING                 1
#define ADDR_LEN_MOVING_STATUS          1
#define ADDR_LEN_PRESENT_PWM            2
#define ADDR_LEN_PRESENT_CURRENT        2
#define ADDR_LEN_PRESENT_VELOCITY       4
#define ADDR_LEN_PRESENT_POSITION       4
#define ADDR_LEN_VELOCITY_TRAJECTORY    4
#define ADDR_LEN_POSITION_TRAJECTORY    4
#define ADDR_LEN_PRESENT_INPUT          2
#define ADDR_LEN_PRESENT_TEMPERATURE    1

#endif /* CONTROL_TABLE_HPP_ */
