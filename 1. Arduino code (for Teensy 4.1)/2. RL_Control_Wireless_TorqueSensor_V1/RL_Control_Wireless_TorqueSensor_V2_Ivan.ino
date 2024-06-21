//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
//FOR THE TORQUE SENSORS: for new sensors, calibrate with a known torque and then modify the gain values in line 140

#include "Serial_Isra.h"
#include "WL_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
/*MOTOR*/
#include <FlexCAN_T4.h>
#include "Motor_Control_Tmotor.h"
#include "ads1292r.h" //FOR THE TORQUE SENSORS
ads1292r torque_sensor1;//FOR THE TORQUE SENSORS
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

/*Filter*/
MovingAverage LTAVx(12);
MovingAverage RTAVx(12);
float f_LTAVx = 0;
float f_RTAVx = 0;

CAN_message_t msgR;

int Motor_ID = 1;
int Motor_ID2 = 2;
int CAN_ID = 3; // Teensy 4.1 CAN bus port

double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

double M1_torque_command = 0;
double M2_torque_command = 0;

int LimitInf = -18;
int LimitSup = 18;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor m1(Motor_ID, CAN_ID);
Motor_Control_Tmotor m2(Motor_ID2, CAN_ID);
/*MOTOR*/

/*Isra Serial Class Setup*/
Serial_Isra Serial_Isra;

/*Sensors Setup*/
IMU imu;

/*Serial Send*/
size_t Send_Length = 11;
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;
uint16_t R_IMUX_int = 0x00;

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00;

uint16_t L_CMD_int16 = 0x7fff;
float L_CMD_serial = 0.0;

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial = 0.0;

float IMUX_float = 0;
float IMU11 = 0;
float IMU22 = 0;
float IMU33 = 0;
float IMU44 = 0;

/* Time control*/
// unsigned long Delta_T1 = 35;  //Looks like increasing this improves stability, but mkaes the torque less smooth
// unsigned long t_i, t_pr1;
// unsigned long beginning = 0;
double t;
// double HZ = 1.0 / (Delta_T1 / 1000.0);


//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;  // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int L_leg_IMU_angle = 0;                //left knee angle
int R_leg_IMU = 0;
int L_motor_torque = 0;
int R_motor_torque = 0;
int L_motor_torque_desired = 0;
int R_motor_torque_desired = 0;
int L_torque_sensor = 0;
int R_torque_sensor = 0;
int t_teensy = 0;
int M_Selected = 0;
int CtrlMode_Selected = 0;
int GUI_cmd = 0;
double GUI_K = 0.01;
//**************************

void setup() {
  delay(3000);
  Serial.begin(115200);   //115200/9600=12
  Serial2.begin(115200);  //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  Serial_Isra.INIT();
  //#################
  Serial.println("SETUP DONE");
  Serial.print("Controller executed at ");
  Serial.print(String(cyclespersec_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(cyclespersec_ble));
  Serial.println(" Hz");
  //####################
  initial_CAN();
  initial_M1();
  initial_M2();
  delay(100);
  IMUSetup();
  t_0 = micros();

  torque_sensor1.Torque_sensor_initial(); //initial the torque sensor see ads1292r.cpp.//FOR THE TORQUE SENSORS
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) *1.97, 0.0003446 * (-1) * 1.8); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.//FOR THE TORQUE SENSORS. M1 Left (tape closer to header) - The first in this list.
  torque_sensor1.Torque_sensor_offset_calibration();//FOR THE TORQUE SENSORS
  delay(1000);//FOR THE TORQUE SENSORS

  SPI1.setMOSI(26);//FOR THE TORQUE SENSORS
  SPI1.setMISO(1);//FOR THE TORQUE SENSORS
  SPI1.setSCK(27);//FOR THE TORQUE SENSORS
}

void loop() {

  imu.READ();
  Serial_Isra.READ2();
  
  current_time = micros() - t_0;
  t = current_time / 1000000.0;

  if (current_time - previous_time > Tinterval_ctrl_micros) {
    
    if (current_time - previous_time_ble > Tinterval_ble_micros) {

      Receive_ble_Data();
      Transmit_ble_Data(); // send the BLE data

      previous_time_ble = current_time;
    }

    //fakeIMU();
    RealIMU();

    Serial_Isra.WRITE(Send, Send_Length);

    L_CMD_int16 = (Serial_Isra.SerialData2[3] << 8) | Serial_Isra.SerialData2[4];
    L_CMD_serial = Serial_Isra.uint_to_float(L_CMD_int16, -20, 20, 16);

    R_CMD_int16 = (Serial_Isra.SerialData2[5] << 8) | Serial_Isra.SerialData2[6];
    R_CMD_serial = Serial_Isra.uint_to_float(R_CMD_int16, -20, 20, 16);

    M1_torque_command = GUI_K * L_CMD_serial;
    M2_torque_command = GUI_K * -R_CMD_serial;

    int max_allowed_torque = 30; // Safety measurement to limit the commanded torque

    if (abs(M1_torque_command) > max_allowed_torque || abs(M2_torque_command) > max_allowed_torque) {
      M1_torque_command = 0;
      M2_torque_command = 0;
      Serial.println("ERROR 1");
      Serial.println("ERROR 1");
      Serial.println("ERROR 1");
      Serial.println("ERROR 1");
      Serial.println("ERROR 1");
      Serial.println("You have exeded the maximum allowed torque.");
      Serial.println("Yoo must adjust the gains of the controller and redo the whole experiment.");
      RealIMU_Reset();
      Serial_Isra.WRITE(Send, Send_Length);
      Wait(10000);
      delay(1000);
    }

    // if (Serial_Isra.SerialData2[2] == 0x42 && Serial_Isra.SerialData2[7] == 0x43) {

    //   if (abs(M1_torque_command) > max_allowed_torque || abs(M2_torque_command) > max_allowed_torque) {
    //     M1_torque_command = 0;
    //     M2_torque_command = 0;
    //     Serial.println("ERROR 2");
    //     Serial.println("ERROR 2");
    //     Serial.println("ERROR 2");
    //     Serial.println("ERROR 2");
    //     Serial.println("ERROR 2");
    //     RealIMU_Reset();
    //     Serial_Isra.WRITE(Send, Send_Length);
    //     Wait(10000);
    //   }
      
    //   M1_Torque_Control_Example();
    //   Wait(1100); // Increasing this increases stability, but less smooth
    //   M2_Torque_Control_Example();
    //   Wait(1100);

    //   Serial_Isra.SerialData2[2] = 0xff;
    //   Serial_Isra.SerialData2[7] = 0xff;
    // }

    M1_Torque_Control_Example();
    Wait(1100); // Increasing this increases stability, but less smooth
    M2_Torque_Control_Example();
    Wait(1100);

    torque_sensor1.Torque_sensor_read(); //FOR THE TORQUE SENSORS

    print_Data_Ivan();

    previous_time = current_time;
  }
}

void print_Data_Ivan() {
  Serial.print(t); Serial.print(" | ");
  Serial.print(imu.LTx); Serial.print(" | ");
  Serial.print(imu.RTx); Serial.print(" | ");
  Serial.print(m1.torque); Serial.print(" | ");
  Serial.print(L_CMD_serial); Serial.print(" | ");
  Serial.print(torque_sensor1.torque[0]); Serial.print(" | ");//FOR THE TORQUE SENSORS
  Serial.print(torque_sensor1.torque[1]); Serial.print(" | ");//FOR THE TORQUE SENSORS
  Serial.println(" ");
}


void print_Data() {
  //  Serial.print(-20);
  //  Serial.print(" ");
  //  Serial.print(20);
  //  Serial.print(" ");

  //  Serial.print(imu.RTx);
  //  Serial.print(" ");
  //  Serial.print(f_RTAVx);
  //  Serial.print(" ");


  //  Serial.print(IMU22);
  //  Serial.print(" ");
  //Serial.print( t_i / 1000 );
  //Serial.print(" ");
  //Serial.print(imu.RTx / 5);
  //Serial.print(" ");
   Serial.print(imu.RTAVx / 10);
   Serial.print(" ");
  //  Serial.print(M1_torque_command);
  //  Serial.print(" ");
  //Serial.print(R_CMD_serial);//Received by Python from serial usb (commanded by the NN)
  //Serial.print(" ");

  // Serial.print(imu.LTx / 5);
  // Serial.print(" ");
  Serial.print(imu.LTAVx / 10);
   Serial.print(" ");
  Serial.print(m2.torque);//Why is the sign opposite to m1?
  Serial.print(" ");
  //  Serial.print(-M2_torque_command); // The one we send to the motor after R-CMD-SERIAL IS RECEIVED. Should be same as R_CMD_serial, unless saturation
  //  Serial.print(" ");
  //Serial.print(-m2.torque);  //Feedback torque from the motor (estimated with current)
  Serial.print(M2_torque_command);
  Serial.print(" ");

  Serial.print(m1.torque);
  Serial.print(" ");
  Serial.print(M1_torque_command);
  Serial.print(" ");
  Serial.print(LimitInf);
  Serial.print(" ");
  Serial.print(LimitSup);
  Serial.print(" ");

  Serial.println(" ");
}

void print_Data_IMU() {
  Serial.print(-180);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  //  Serial.print(IMU22);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(imu.RTAVx);
  Serial.println(" ");
}

void print_Data_Received() {

  Serial.print(20);
  Serial.print(" ");
  Serial.print(-20);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  Serial.print(" ");
  Serial.print(R_CMD_serial);
  Serial.print(" ");
  Serial.println(" ");
}

void print_data_motor() {
  //  double v1 = 90;
  //  double v2 = -v1;
  //  Serial.print(v1);
  //  Serial.print("   ");
  //Serial.print(current_time);
  Serial.print(" ; ");
  Serial.print(" M1_tor ; "); //M1 is left, M2 is right
  Serial.print(m1.torque);
  Serial.print(" ; M1_cmd ; ");
  Serial.print(M1_torque_command);
  Serial.print(" ; M2_tor ; ");
  Serial.print(m2.torque);
  Serial.print(" ; M2_cmd ; ");
  Serial.print(M2_torque_command);
  Serial.print(" ; M1_pos ; ");
  Serial.print(m1.pos);
  Serial.println(" ;  ");
}

void IMUSetup() {
  imu.INIT();
  delay(500);
  imu.INIT_MEAN();
}

void M1_Torque_Control_Example() {
  p_des = 0;  //dont change this
  v_des = 0;  //dont change this
  kp = 0;     //dont change this
  kd = 0;     //dont change this
  t_ff = M1_torque_command;
  m1.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void M2_Torque_Control_Example() {
  p_des = 0;  //dont change this
  v_des = 0;  //dont change this
  kp = 0;     //dont change this
  kd = 0;     //dont change this
  t_ff = M2_torque_command;
  m2.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}

void initial_M1() {
  //m1.initial_CAN();
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(1000);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  position_command = 0;
  M1_Position_Control_Example();
  Serial.println("M1 Done");
  delay(100);
}

void initial_M2() {
  //m2.initial_CAN();
  m2.exit_control_mode();
  delay(200);
  m2.exit_control_mode();
  delay(1000);
  m2.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  position_command = 0;
  M2_Position_Control_Example();
  Serial.println("M2 Done");
  delay(100);
}

void receive_CAN_data() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);
    int id = msgR.buf[0];
    //Serial.print(msgR.id, HEX );
    if (id == Motor_ID) {
      m1.unpack_reply(msgR);
    }
    if (id == Motor_ID2) {
      m2.unpack_reply(msgR);
    }
  }
}

void fakeIMU() {
  IMU11 = 150.0 * sin(t / 5.0);
  IMU22 = 150.0 * cos(t / 5.0);
  IMU33 = 700.0 * sin(t / 5.0);
  IMU44 = 700.0 * cos(t / 5.0);

  L_IMUX_int = Serial_Isra.float_to_uint(IMU11, -180, 180, 16);
  R_IMUX_int = Serial_Isra.float_to_uint(IMU22, -180, 180, 16);

  L_IMUV_int = Serial_Isra.float_to_uint(IMU33, -800, 800, 16);
  R_IMUV_int = Serial_Isra.float_to_uint(IMU44, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void RealIMU() {
  f_LTAVx = LTAVx.addSample(imu.LTAVx);
  f_RTAVx = RTAVx.addSample(imu.RTAVx);

  L_IMUX_int = Serial_Isra.float_to_uint(imu.LTx, -180, 180, 16);
  R_IMUX_int = Serial_Isra.float_to_uint(imu.RTx, -180, 180, 16);

  //  L_IMUV_int = Serial_Isra.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Isra.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Isra.float_to_uint(f_LTAVx, -800, 800, 16);
  R_IMUV_int = Serial_Isra.float_to_uint(f_RTAVx, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}


void RealIMU_Reset() {
  float reset_imu = 0;

  L_IMUX_int = Serial_Isra.float_to_uint(reset_imu, -180, 180, 16);
  R_IMUX_int = Serial_Isra.float_to_uint(reset_imu, -180, 180, 16);

  //  L_IMUV_int = Serial_Isra.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Isra.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Isra.float_to_uint(reset_imu, -800, 800, 16);
  R_IMUV_int = Serial_Isra.float_to_uint(reset_imu, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void M1_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  m1.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void M2_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  m2.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}

//float mAvg(float x){
//
//  /* Begin
//  #include "MovingAverage.h"
//  MovingAverage filter(12);
//  */
//
//  float avg=filter.addSample(x);
//  return avg;
//  }

void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          M_Selected        = data_rs232_rx[4];
          CtrlMode_Selected = data_rs232_rx[6];
          GUI_cmd           = data_rs232_rx[8];

          GUI_K = GUI_cmd / 100.0;

          Serial.print("| Motor ");
          Serial.print(M_Selected, DEC); // This contains the motor number
          Serial.print(" selected | Control Mode ");
          Serial.print(CtrlMode_Selected, DEC); // This contains the Control mode
          Serial.print(" selected | Command ");
          //Serial.print(GUI_cmd, DEC); // This contains the desired command
          Serial.print(GUI_K);
          Serial.println(" sent |");
        }
      }
    }
  }
}

void Transmit_ble_Data(){
  t_teensy = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU = imu.RTx * 100;
  L_motor_torque = m1.torque * 100;
  R_motor_torque = m2.torque * 100;
  L_motor_torque_desired = M1_torque_command * 100;
  R_motor_torque_desired = M2_torque_command * 100;
  L_torque_sensor = torque_sensor1.torque[0] * 100;
  R_torque_sensor = torque_sensor1.torque[1] * 100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = L_leg_IMU_angle;
  data_ble[4]  = L_leg_IMU_angle >> 8;
  data_ble[5]  = R_leg_IMU;
  data_ble[6]  = R_leg_IMU >> 8;
  data_ble[7]  = L_motor_torque;
  data_ble[8]  = L_motor_torque >> 8;
  data_ble[9]  = R_motor_torque;
  data_ble[10] = R_motor_torque >> 8;
  data_ble[11] = L_motor_torque_desired;
  data_ble[12] = L_motor_torque_desired >> 8;
  data_ble[13] = R_motor_torque_desired;
  data_ble[14] = R_motor_torque_desired >> 8;
  data_ble[15] = t_teensy;
  data_ble[16] = t_teensy >> 8;
  data_ble[17] = L_torque_sensor;
  data_ble[18] = L_torque_sensor >> 8;
  data_ble[19] = R_torque_sensor;
  data_ble[20] = R_torque_sensor >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);
  //Serial7.write(data_ble, datalength_ble);
  //Serial.print("Transmit Data Function Executed");
}
