/*******************************************************************************
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

/********************************************************************************
* @author DY
* @brief
*   This code consists of 2 thread, serial-read and serial-write RTOS system.
*   Default OP_MODE: Extended-Position
*   Default Position P Gain: 3000
*   Default Position I Gain: 0
*   Default Position D Gain: 3000

* @protocol
*   STX: /
*   CTX: ;
*   READ: /Functinos name(Dynamixel2Arduino), arg1, arg2, ...;
*     @example 
*       If you want to call 'dxl.setOperatingMode(1, OP_POSITION)',
*       you can send '/setOperatingMode,1,3;' to OpenRB-150
*     @example
*       If you want to see the motor states with protocolc:\Users\daeyun\Desktop\github_repositories_Bigyuun\Dynamixel2Arduino-RTOS\src\actuator.h, you can send '/monitorOff;'.
*         - [Thread-Serial Write] MOTOR #1 -> Present Pos(Deg)= 87.91 / Current(mA) = 0.00 - MOVING: 0
*       Else if you want to see without protocol, you can send '/monitorOn;'
*         - /1,0,0,87.912003,0.000000,0.000000;
*     @example
*       If you want to check the task information about each multi-thread each,
*       you can send '/task';
*   WRITE: /motor_id, errcode, MOVING, PresentPosition(degree), PresentVelocity, PresentCurrent;
*     @example /1,0,0,87.912003,0.000000,8.000000;
* @note
********************************************************************************/

/** custom **/
#include "definition.hpp"

//**************************************************************************
// global variables
//**************************************************************************
// Debug
bool monitoring_flag = true;

// Dynamixel
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
uint8_t g_num_of_motors = 0;
uint8_t DXL_ID_LIST[DXL_ID_CNT];

// int8_t g_torque_enable[DXL_ID_CNT];
// int8_t g_moving[DXL_ID_CNT];
// int16_t g_present_current[DXL_ID_CNT];
// int32_t g_present_velocity[DXL_ID_CNT];
// int32_t g_present_position[DXL_ID_CNT];

/**
* @author DY
* @note
    Declare each structures for read multi-data
    In this case,
      - torque_enable
      - moving
      - present_current
      - present_velocity
      - present_position
      - hardware_error
*/
SyncReadData_t sr_data[DXL_ID_CNT];
const uint16_t user_pkt_buf_cap = 128;

uint8_t user_pkt_buf_torque_enable[user_pkt_buf_cap];
DYNAMIXEL::InfoSyncReadInst_t sr_infos_torque_enable;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_torque_enable[DXL_ID_CNT];

uint8_t user_pkt_buf_moving[user_pkt_buf_cap];
DYNAMIXEL::InfoSyncReadInst_t sr_infos_moving;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_moving[DXL_ID_CNT];

// For reading cur & vel & pos because its addresses are side by side
uint8_t user_pkt_buf_present_cvp[user_pkt_buf_cap];
DYNAMIXEL::InfoSyncReadInst_t sr_infos_present_cvp;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_present_cvp[DXL_ID_CNT];

// RTOS handler
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_serialreadTask;
TaskHandle_t Handle_serialwriteTask;
// TaskHandle_t Handle_DynamixelTask;

// synchronizing Mutex
SemaphoreHandle_t serial_mutex;

/**
* @author DY
* @brief Serial read thread for implementing the command based on protocol.
*/
static void thread_serial_read( void *pvParameters ) 
{
  SERIAL.println("Serial Read node is up.");

  while(true){
    if (SERIAL.available() <= 0) {
      continue;
    }

    String msg = "";
    while (SERIAL.available() > 0) {
      char c = SERIAL.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      msg += c;
    }

    // received messages update
    // String msg = SERIAL.readString();
    SERIAL.print("recv msg: ");
    SERIAL.println(msg);
    std::vector<String> v_command;
    bool success = parsing_command(msg, v_command);
    if (!success)
      continue;

    // xSemaphoreTake(serial_mutex, 100 / portTICK_PERIOD_MS);

    if (v_command[0] == "monitorOn") monitoring_flag = true;
    else if (v_command[0] == "monitorOff")  monitoring_flag = false;
    else if (v_command[0] == "task") print_task_info();
    else {
      dynamixel_function_callback(v_command);
    }

    // xSemaphoreTake(serial_mutex, 100 / portTICK_PERIOD_MS);
    SERIAL.print("[Thread-Serial READ] Command: [size]: ");
    SERIAL.print(v_command.size());
    SERIAL.print(" / [value]: ");
    if (success) {
      for (int i = 0; i < v_command.size(); i++) {
        Serial.print(v_command[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Error parsing and storing values!");
    }
    SERIAL.flush();
    // xSemaphoreGive(serial_mutex);

    vTaskDelay( 1 / portTICK_PERIOD_MS ); // Never delete
  }

  SERIAL.println("*** [Thread-Serial READ] THREAD DELETE");
}


/**
* @author DY
* @brief Serial write thread for echo of motor's states
*/
static void thread_serial_write( void *pvParameters ) 
{
  SERIAL.println("Serial write node is up.");
  
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  // uint8_t recv_cnt_te = dxl.syncRead(&sr_infos_torque_enable);
  // uint8_t recv_cnt_m = dxl.syncRead(&sr_infos_moving);
  // uint8_t recv_cnt_cvp = dxl.syncRead(&sr_infos_present_cvp);

  uint8_t recv_cnt_te = 0;
  uint8_t recv_cnt_m = 0;
  uint8_t recv_cnt_cvp = 0;


  uint8_t torque_enable[DXL_ID_CNT];
  uint8_t moving[DXL_ID_CNT] ;
  float present_cur[DXL_ID_CNT];
  float present_vel[DXL_ID_CNT];
  float present_pos[DXL_ID_CNT];

  // TEST
  // uint8_t recv_cnt = dxl.syncRead(&sr_infos);

  while(1){

    DelayMsUntil(&lastWakeTime, 1000/SERIAL_WRITE_FREQUENCY);

    recv_cnt_te = dxl.fastSyncRead(&sr_infos_torque_enable);
    recv_cnt_m = dxl.fastSyncRead(&sr_infos_moving);
    recv_cnt_cvp = dxl.fastSyncRead(&sr_infos_present_cvp);

    if(recv_cnt_te > 0) {
      for(uint8_t i = 0; i < recv_cnt_te; i++) {
        torque_enable[i] = sr_data[i].torque_enable;
      }
    } else {
      SERIAL.print("SyncRead Torque Enable Fail, Lib error code: ");
      SERIAL.println(dxl.getLastLibErrCode());
    }

    if(recv_cnt_m > 0) {
      for(uint8_t i = 0; i < recv_cnt_m; i++) {
        moving[i] = sr_data[i].moving;
      }
    } else {
      SERIAL.print("SyncRead moving Fail, Lib error code: ");
      SERIAL.println(dxl.getLastLibErrCode());
    }

    if(recv_cnt_cvp > 0) {
      for(uint8_t i = 0; i < recv_cnt_cvp; i++) {
        present_cur[i] = sr_data[i].present_current;
        present_vel[i] = sr_data[i].present_velocity;
        present_pos[i] = sr_data[i].present_position;
      }
    } else {
      SERIAL.print("SyncRead Current & Velocity & Position Fail, Lib error code: ");
      SERIAL.println(dxl.getLastLibErrCode());
    }

    if (monitoring_flag) {
      for(int i=0; i<DXL_ID_CNT; i++){
        // xSemaphoreTake(serial_mutex, portMAX_DELAY);
        // xSemaphoreTake(serial_mutex, 100 / portTICK_PERIOD_MS);
        SERIAL.print("ID: ");
        SERIAL.print(i + 1);
        SERIAL.print(", Torque Enable: ");
        SERIAL.print(torque_enable[i]);
        SERIAL.print(", Moving: ");
        SERIAL.print(moving[i]);
        SERIAL.print(", Cur: ");
        SERIAL.print(present_cur[i]);
        SERIAL.print(", Vel: ");
        SERIAL.print(present_vel[i]);
        SERIAL.print(", Pos: ");
        SERIAL.print(present_pos[i]);
        SERIAL.println();
        // xSemaphoreGive(serial_mutex);
      }
    }
    else {
      for(int i=0; i<DXL_ID_CNT; i++){
        char msg[50];
        sprintf(msg, "/%d,%d,%d,%f,%f,%f;", i+1, torque_enable[i], moving[i], present_cur[i], present_vel[i], present_pos[i]);
        // xSemaphoreTake(serial_mutex, portMAX_DELAY);
        SERIAL.println(msg);
        SERIAL.flush();
        // xSemaphoreGive(serial_mutex);
      }
      
    }
  
    vTaskDelay( 5 / portTICK_PERIOD_MS ); // Never delete
  }
  SERIAL.println("*** [Thread-Serial Write] THREAD DELETE");
}

// static void thread_Dynamixel_node( void *pvParameters ) 
// {
//   SERIAL.println("Dynamixel node is up.");
//   while(true){
//     /** 
//       * write the code
//       */
//     dxl.getPresentPosition(1, UNIT_DEGREE);
//     dxl.getPresentCurrent(1, UNIT_MILLI_AMPERE);
//     // xSemaphoreTake(seric:\Users\daeyun\Desktop\github_repositories_Bigyuun\Dynamixel2Arduino-RTOS\examples\RTOS\RTOS_sync_read_dev\definition.hppal_mutex, portMAX_DELAY);
//     // SERIAL.print("[Thread-Dynamixel State] Present Current(mA)= ");
//     // SERIAL.println(dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE));
//     // SERIAL.flush();
//     // xSemaphoreGive(serial_mutex);
//     vTaskDelay( 10 / portTICK_PERIOD_MS );
//   }
//   SERIAL.println("*** [Thread-Dynamixel] THREAD DELETE");
// }

/**********************************************************************
* @author DY
* @brief
    - Serial communication setting
* @note
*/
void serial_init()
{
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  SERIAL.setTimeout(SERIAL_SETTIMEOUT);
  SERIAL.begin(SERIAL_BAUDRATE);
  // SERIAL.begin(57600);
  while(!SERIAL);

  

  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  serial_mutex = xSemaphoreCreateMutex();
  if (serial_mutex == NULL){
    SERIAL.println("Mutex can not be created.");
  }
  else{
    SERIAL.println("Mutex is created completely.");
  }

  // check controltable mapping with string
  std::string searchKey = "REGISTERED";
  ControlTableItemIndex index = findIndex(searchKey);
  SERIAL.print("The index of ");
  SERIAL.print(searchKey.c_str());
  SERIAL.print(" is ");
  SERIAL.print(index);
  SERIAL.println();
  delay(1000);
}

/**
* @author DY
* @brief
    - connect to dynamixel motors on OpenRB-150
    - find total number of motors
    - default setting depends on 'definition.hpp'
* @note
*/
void dynamixel_init()
{
  bool success;
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BAUDRATE);

  SERIAL.print("DXL_BAUDRATE: ");
  SERIAL.println(DXL_BAUDRATE);
  SERIAL.print("DYNAMIXEL_PROTOCOL_VERSION: ");
  SERIAL.println(DYNAMIXEL_PROTOCOL_VERSION);
  SERIAL.print("OPERATION_MODE: ");
  SERIAL.println(OP_MODE);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  if(!dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION)) SERIAL.println("setPortProtocolVersion() Fail.");
  
  //////////////////////////////////////////////////////////////////
  // Find motors
  //////////////////////////////////////////////////////////////////
  char str_num_of_motors[50];
  SERIAL.print("Searching motors... - ID: ");
  for (int i=1; i<=252; i++) {
    static std::vector<uint8_t> ids;
    bool success = dxl.ping(i);
    if (success) {
      ids.push_back(i); 
      SERIAL.print(i);
      SERIAL.print(" / ");
      g_num_of_motors++;
    }
  }
  sprintf(str_num_of_motors, "Found %d motors.", g_num_of_motors);
  SERIAL.println(str_num_of_motors);
  if(g_num_of_motors != DXL_ID_CNT) {SERIAL.println("Not match number of motors between founding & declaring");}
  if (g_num_of_motors == DXL_ID_CNT) {
    SERIAL.println("Setting & Searching number of motors is same... OK");
  }
  else {
    SERIAL.println("Setting & Searching number of motors is not same... WARNING");
  }

  //////////////////////////////////////////////////////////////////
  // set default
  //////////////////////////////////////////////////////////////////
  for (int i=1; i<=DXL_ID_CNT; i++) {
    // Get DYNAMIXEL information
    if(!dxl.ping(i)) SERIAL.println("ping() Fail.");  
    if(!dxl.setOperatingMode(i, OP_MODE)) SERIAL.println("setOperatingMode() Fail.");
    
    // Turn off torque when configuring items in EEPROM area
    if(!dxl.torqueOff(i)) SERIAL.println("torqueOff() Fail.");  
    // if(!dxl.torqueOn(i)) SERIAL.println("torqueOn() Fail.");  

    // Set Position PID Gains
    if(!dxl.writeControlTableItem(POSITION_P_GAIN, i, DXL_POSITION_P_GAIN)) SERIAL.println("writeControlTableItem(POSITION_P_GAIN) Fail.");
    if(!dxl.writeControlTableItem(POSITION_I_GAIN, i, DXL_POSITION_I_GAIN)) SERIAL.println("writeControlTableItem(POSITION_I_GAIN) Fail.");
    if(!dxl.writeControlTableItem(POSITION_D_GAIN, i, DXL_POSITION_D_GAIN)) SERIAL.println("writeControlTableItem(POSITION_D_GAIN) Fail.");
  }
  dxl.torqueOff(BROADCAST_ID);


  //////////////////////////////////////////////////////////////////
  // Fill the members of structure to syncRead using external user packet buffer
  //////////////////////////////////////////////////////////////////

  // set motor ID
  for (int i=0; i<DXL_ID_CNT; i++) {
    DXL_ID_LIST[i] = i + 1;
  }

  // torque_enable =============================================
  sr_infos_torque_enable.packet.p_buf = user_pkt_buf_torque_enable;
  sr_infos_torque_enable.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos_torque_enable.packet.is_completed = false;
  sr_infos_torque_enable.addr = ADDR_TORQUE_ENABLE;
  sr_infos_torque_enable.addr_length = ADDR_LEN_TORQUE_ENABLE;
  sr_infos_torque_enable.p_xels = info_xels_sr_torque_enable;
  sr_infos_torque_enable.xel_count = 0;  
  for(int i=0; i<DXL_ID_CNT; i++){
    info_xels_sr_torque_enable[i].id = DXL_ID_LIST[i];
    info_xels_sr_torque_enable[i].p_recv_buf = (uint8_t*)&sr_data[i].torque_enable;
    sr_infos_torque_enable.xel_count++;
  }
  sr_infos_torque_enable.is_info_changed = true;

  // moving =============================================
  sr_infos_moving.packet.p_buf = user_pkt_buf_moving;
  sr_infos_moving.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos_moving.packet.is_completed = false;
  sr_infos_moving.addr = ADDR_MOVING;
  sr_infos_moving.addr_length = ADDR_LEN_MOVING;
  sr_infos_moving.p_xels = info_xels_sr_moving;
  sr_infos_moving.xel_count = 0;  
  for(int i=0; i<DXL_ID_CNT; i++){
    info_xels_sr_moving[i].id = DXL_ID_LIST[i];
    info_xels_sr_moving[i].p_recv_buf = (uint8_t*)&sr_data[i].moving;
    sr_infos_moving.xel_count++;
  }
  sr_infos_moving.is_info_changed = true;

  // present_cur & vel & pos =============================================
  sr_infos_present_cvp.packet.p_buf = user_pkt_buf_present_cvp;
  sr_infos_present_cvp.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos_present_cvp.packet.is_completed = false;
  sr_infos_present_cvp.addr = ADDR_PRESENT_CURRENT;
  sr_infos_present_cvp.addr_length = ADDR_LEN_PRESENT_CURRENT + ADDR_LEN_PRESENT_VELOCITY + ADDR_LEN_PRESENT_POSITION;
  sr_infos_present_cvp.p_xels = info_xels_sr_present_cvp;
  sr_infos_present_cvp.xel_count = 0;  
  for(int i=0; i<DXL_ID_CNT; i++){
    info_xels_sr_present_cvp[i].id = DXL_ID_LIST[i];
    info_xels_sr_present_cvp[i].p_recv_buf = (uint8_t*)&sr_data[i].present_current;
    sr_infos_present_cvp.xel_count++;
  }
  sr_infos_present_cvp.is_info_changed = true;

  SERIAL.println("Dynamixel initilaizing... OK");
}

/**
* @author Bigyuun
* @brief parsing command and its arguments
* @protocol /function_name,argument1,argument2,...;
*/
bool parsing_command(String msg, std::vector<String> & command_vector)
{
  
  command_vector.clear();

  // find '/'
  int ipos0 = msg.indexOf('/');
  if (ipos0 < 0)
    return false;
  // find ';'
  int ipos1 = msg.indexOf(';');
  if (ipos1 < 0)
    return false;

  // get valid message
  String valid_msg = msg.substring(ipos0 + 1, ipos1);

  // get command and arguments
  int idx_start = 0;
  int idx_comma = 0;
  while(true)
  {
    idx_comma = valid_msg.indexOf(',', idx_start);
    if (idx_comma != -1) {
      command_vector.push_back(valid_msg.substring(idx_start, idx_comma));
      idx_start = idx_comma + 1;
    } else {  // 마지막 문자열인 경우
      command_vector.push_back(valid_msg.substring(idx_start));
      return true; // 값 저장 성공
    }
  }
  // error
  return false;
}


/**
  * @source https://learn.microsoft.com/en-us/cpp/cpp/constexpr-cpp?view=msvc-170
  * @brief
  * The keyword constexpr was introduced in C++11 and improved in C++14. It means constant expression. Like const, it can be applied to variables: A compiler error is raised when any code attempts to modify the value. Unlike const, constexpr can also be applied to functions and class constructors. constexpr indicates that the value, or return value, is constant and, where possible, is computed at compile time.
  * A constexpr integral value can be used wherever a const integer is required, such as in template arguments and array declarations. And when a value is computed at compile time instead of run time, it helps your program run faster and use less memory.
  * To limit the complexity of compile-time constant computations, and their potential impacts on compilation time, the C++14 standard requires the types in constant expressions to be literal types.
  * @note DY-참고로 constexpr은 람다함수로 사용할 수 없어서 외부사용자 함수로써 선언해뒀음.
  */
constexpr unsigned int HashCode(const char* str)
{
    return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * HashCode(str + 1) : 8603;
}
// [출처] [C++] switch 문에서 문자열 사용하기 (Feat. constexpr)|작성자 데브머신
// https://blog.naver.com/devmachine/220952781191

/**
  * @brief check the command vector and implement the function of Dynamixel2Arduino
  * @protocol v_command = [command, arg1, arg2, ...]
  */
bool dynamixel_function_callback(std::vector<String> v_command)
{
  uint8_t len = v_command.size();
  String command = v_command[0];
  bool success = false;

  switch(HashCode(command.c_str()))
  {
    case HashCode("begin"):
      /**
        * @example
        * const int DXL_DIR_PIN = 2;
        * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
        * dxl.begin(57600);
        */

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.begin(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.print("begin() -> success: ");
      SERIAL.println(success);
      break;
      
    case HashCode("getPortBaud"):
      /**
        * @example  dxl.getPortBaud()
        */

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 0) { dxl.getPortBaud(); return true; }
        else return false;
      }(len);
      SERIAL.print("getPortBaud() -> success: ");
      SERIAL.println(success);
      break;

    case HashCode("ping"):
      /**
        * @example  dxl.ping(1);
        */
      SERIAL.print("ping()");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.ping(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("scan"):
      /**
        * @example dxl.scan();
        */
      SERIAL.print("scan() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 0) { dxl.scan(); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("getModelNumber"):
      /**
        * @example dxl.getModelNumber(1)
        */
      SERIAL.print("getModelNumber() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.getModelNumber(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("setID"):
      /**
        * @param id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param new_id : A new ID to assign to the DYNAMIXEL. Data type : unsigned int8
        * @example dxl.setID(1, 2);
        */
      SERIAL.print("setID() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setID(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("setProtocol"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  version : Protocol version to use. Data type : float
        * @example  dxl.setProtocol(1, 1.0);
        */
      SERIAL.print("setProtocol() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setProtocol(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("setBaudrate"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  baudrate : Communication speed(baudrate) to use. Data type : unsigned int32
        * @example  dxl.setBaudrate(1, 57600);
        */
      SERIAL.print("setBaudrate() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setBaudrate(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("torqueOn"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @example  dxl.torqueOn(1);
        */
      SERIAL.print("torqueOn() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.torqueOn(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("torqueOff"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @example  dxl.torqueOff(1);
        */
      SERIAL.print("torqueOff() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.torqueOff(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("ledOn"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @example  dxl.ledOn(1);
        */
      SERIAL.print("ledOn() -> success:");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.ledOn(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("ledOff"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @example
        */
      SERIAL.print("ledOff() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.ledOff(v_command[1].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;

    case HashCode("setOperatingMode"):
      /** 
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  mode : Operating mode to use. Data type : unsigned int8
        *  Decimal Library	Defined	                    Mode
        *  0	     (3)      OP_POSITION                	Position Control Mode
        *  1	     (4)      OP_EXTENDED_POSITION        Extended Position Control Mode
        *  2	     (5)      OP_CURRENT_BASED_POSITION   Current-based Position Control Mode
        *  3	     (1)      OP_VELOCITY                	Velocity Control mode
        *  4	     (16)     OP_PWM               	      PWM Control Mode
        *  5	     (0)      OP_CURRENT               	  Curernt Control Mode
        * @example  dxl.setOperatingMode(1, OP_POSITION);
        */
      SERIAL.print("setOperatingMode() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setOperatingMode(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("setGoalPosition"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  value : Goal Position value in encoder value(default) or angle in degree. Data type : float
        * @param  unit : Types of value. Data type : unsigned int8
        *   For encoder value : UNIT_RAW (default)
        *   For angle value : UNIT_DEGREE
        * @example
        *   dxl.setGoalPosition(1, 512); //use default encoder value
        *   dxl.setGoalPosition(2, 45.0, UNIT_DEGREE); //use angle in degree
        */
      SERIAL.print("setGoalPosition() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setGoalPosition(v_command[1].toInt(), v_command[2].toFloat()); return true; }
        else if (num_arg == 3) { dxl.setGoalPosition(v_command[1].toInt(), v_command[2].toFloat(), v_command[3].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;

    case HashCode("getPresentPosition"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  unit : Types of value. Data type : unsigned int8
        *   For encoder value : UNIT_RAW (default)
        *   For angle value : UNIT_DEGREE
        * @example
        *   Serial.print(dxl.getPresentPosition(1)); //use default encoder value
        *   Serial.print(dxl.getPresentPosition(1, UNIT_DEGREE)); //use angle in degree
        */
      SERIAL.print("getPresentPosition() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.getPresentPosition(v_command[1].toInt()); return true; }
        else if (num_arg == 2) { dxl.getPresentPosition(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;

    case HashCode("setGoalVelocity"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  value : Goal Velocity value in raw value(default) or RPM or percentage. Data type : float
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For RPM value : UNIT_RPM
        *   For Percentage value : UNIT_PERCENT
        * @example  
        *   dxl.setGoalVelocity(1, 100); //use default raw value
        *   dxl.setGoalVelocity(2, 10.5, UNIT_RPM); //use RPM
        *   dxl.setGoalVelocity(3, 15.0, UNIT_PERCENT); //use percentage (-100 ~ 100 %)
        */
      SERIAL.print("setGoalVelocity() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setGoalVelocity(v_command[1].toInt(), v_command[2].toFloat()); return true; }
        else if (num_arg == 3) { dxl.setGoalVelocity(v_command[1].toInt(), v_command[2].toFloat(), v_command[3].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    
    case HashCode("getPresentVelocity"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For RPM value : UNIT_RPM
        *   For Percentage value : UNIT_PERCENT
        * @example
        *   Serial.print(dxl.getPresentVelocity(1)); //use default raw value
        *   Serial.print(dxl.getPresentVelocity(2, UNIT_RPM)); //use RPM
        *   Serial.print(dxl.getPresentVelocity(3, UNIT_PERCENT)); //use percentage (-100 ~ 100 %)
        */
      SERIAL.print("getPresentVelocity() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.getPresentVelocity(v_command[1].toInt()); return true; }
        else if (num_arg == 2) { dxl.getPresentVelocity(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      

    case HashCode("setGoalPWM"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  value : Goal PWM value in raw value(default) or percentage. Data type : float
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For Percentage value : UNIT_PERCENT
        * @example  
        *   dxl.setGoalPWM(1, 100); //use default raw value
        *   dxl.setGoalPWM(2, 50.0, UNIT_PERCENT); //use percentage (-100 ~ 100 %)
        */
      SERIAL.print("setGoalPWM() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setGoalPWM(v_command[1].toInt(), v_command[2].toFloat()); return true; }
        else if (num_arg == 3) { dxl.setGoalPWM(v_command[1].toInt(), v_command[2].toFloat(), v_command[3].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("getPresentPWM"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For Percentage value : UNIT_PERCENT
        * @example
        *   Serial.print(dxl.getPresentPWM(1)); //use default raw value
        *   Serial.print(dxl.getPresentPWM(2, UNIT_PERCENT)); //use percentage (-100 ~ 100 %)
        */
      SERIAL.print("getPresentPWM() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.getPresentPWM(v_command[1].toInt()); return true; }
        else if (num_arg == 2) { dxl.getPresentPWM(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("setGoalCurrent"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  value : Goal Current value in raw value(default) or milli amphere or percentage. Data type : float
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For milli amphere(mA) : UNIT_MILLI_AMPERE
        *   For Percentage value : UNIT_PERCENT
        * @example
        *   dxl.setGoalCurrent(1, 100); //use default raw value
        *   dxl.setGoalCurrent(2, 1000.0, UNIT_MILLI_MAPERE); //use mA
        *   dxl.setGoalCurrent(3, 50.0, UNIT_PERCENT); //use percentage (-100 ~ 100 %)
        */
      SERIAL.println("setGoalCurrent() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.setGoalCurrent(v_command[1].toInt(), v_command[2].toFloat()); return true; }
        else if (num_arg == 3) { dxl.setGoalCurrent(v_command[1].toInt(), v_command[2].toFloat(), v_command[3].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("getPresentCurrent"):
      /**
        * @param  id : ID of a specific DYNAMIXEL. Data type : unsigned int8
        * @param  unit : Types of value. Data type : unsigned int8
        *   For raw value : UNIT_RAW (default)
        *   For milli amphere(mA) : UNIT_MILLI_AMPERE
        *   For Percentage value : UNIT_PERCENT      
        * @example
        *   Serial.print(dxl.getPresentCurrent(1)); //use default raw value
        *   Serial.print(dxl.getPresentCurrent(2, UNIT_MILLI_MAPERE)); //use mA
        *   Serial.print(dxl.getPresentCurrent(3, UNIT_PERCENT)); //use percentage (-100 ~ 100 %)
        */
      SERIAL.print("getPresentCurrent() -> success: ");
      
      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 1) { dxl.getPresentCurrent(v_command[1].toInt()); return true; }
        else if (num_arg == 2) { dxl.getPresentCurrent(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("readControlTableItem"):
      /**
        * @example
        */
      SERIAL.print("readControlTableItem() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 2) { dxl.readControlTableItem(v_command[1].toInt(), v_command[2].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;
      
    case HashCode("writeControlTableItem"):
      /**
        * @example
        */
      SERIAL.print("writeControlTableItem() -> success: ");

      // Lambda Function: Check the number of arguments & implement
      success = [v_command](uint8_t len) -> bool
      {
        uint8_t num_arg = v_command.size() - 1;
        if (num_arg == 3) { dxl.writeControlTableItem(v_command[1].toInt(), v_command[2].toInt(), v_command[3].toInt()); return true; }
        else return false;
      }(len);
      SERIAL.println(success);
      break;

    default:
      break;
  }
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
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
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

  // measurement = uxTaskGetStackHighWaterMark( Handle_DynamixelTask );
  // SERIAL.print("Dynamixel Task: ");
  // SERIAL.println(measurement);

  measurement = uxTaskGetStackHighWaterMark( Handle_serialwriteTask );
  SERIAL.print("Serial Write Task: ");
  SERIAL.println(measurement);

  measurement = uxTaskGetStackHighWaterMark( Handle_serialreadTask );
  SERIAL.print("Serial Read Task: ");
  SERIAL.println(measurement);
  
  SERIAL.println("****************************************************");
  SERIAL.flush();
  xSemaphoreGive(serial_mutex);
}



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

/**
* @author DY
* @brief
*   Make periodic function
* @code
*   TickType_t lastWakeTime = xTaskGetTickCount();
*   int period_ms = 10 
*   while(1){
*     DelayMsUntil(&lastWakeTime, period_ms);
*     // Operation
*   }
* @endcode
*/
void DelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
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
    512,
    NULL,
    tskIDLE_PRIORITY + 2,
    &Handle_serialwriteTask);
  
  xTaskCreate(thread_serial_read,
    "Task Serial Read",
    512,
    NULL,
    tskIDLE_PRIORITY + 1,
    &Handle_serialreadTask);

  // xTaskCreate(thread_Dynamixel_node,
  //   "Task Dynamixel",
  //   256,
  //   NULL,
  //   tskIDLE_PRIORITY + 0,
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

