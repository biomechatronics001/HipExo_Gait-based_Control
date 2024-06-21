/*Libraries*/
#include "WL_IMU.h"
#include <Arduino.h>
//#include "MovingAverage.h"
#include "CIC.h"
#include "Serial_Isra.h"
#include <SD.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
#include "ads1292r.h" //FOR THE TORQUE SENSORS
ads1292r torque_sensor1;//FOR THE TORQUE SENSORS
/*
const int chipSelect = BUILTIN_SDCARD;
#define namesd "isratest4545.txt"*/

/*Isra Serial Class Setup*/
Serial_Isra Serial_Isra;

/*Serial Send*/
size_t Send_Length = 30;
char Send[30] = {0x31, 0x32, 0x32, 0x33, 0x33,
                 0x32, 0x32, 0x32, 0x32, 0x32,
                 0x32, 0x32, 0x32, 0x32, 0x32,
                 0x32, 0x32, 0x32, 0x32, 0x32,
                 0x32, 0x32, 0x32, 0x32, 0x32,
                 0x32, 0x32, 0x32, 0x32, 0x33,
                };

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;
uint16_t R_IMUX_int = 0x00;

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00;

uint16_t L_tor_int = 0x00;
uint16_t R_tor_int = 0x00;

uint16_t L_cmd_int = 0x00;
uint16_t R_cmd_int = 0x00;

uint16_t GP_IMU_L_int = 0x00;
uint16_t GP_IMU_R_int = 0x00;


/*MOTOR*/
#include <FlexCAN_T4.h>
#include "Motor_Control_Tmotor.h"
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

/*Filter*/
//MovingAverage LTAVx(12);
//MovingAverage RTAVx(12);
float f_LTAVx = 0;
float f_RTAVx = 0;

CAN_message_t msgR;

/*MOTOR*/
int Motor_ID = 1;
int Motor_ID2 = 2;
int CAN_ID = 3;

double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

double M1_torque_command = 0;
double M2_torque_command = 0;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor m1(Motor_ID, CAN_ID);
Motor_Control_Tmotor m2(Motor_ID2, CAN_ID);
/*MOTOR*/

/*Sensors Setup*/
IMU imu;

/*CIC Setup*/
CIC cic;

/* Time control*/
unsigned long Delta_T1 = 24000;
unsigned long t_i = 0;
unsigned long t_pr1 = 0;
unsigned long t_pr2 = 0;
unsigned long t_pr3 = 0;
unsigned long beginning = 0;
double t;
double HZ = 1.0 / (Delta_T1 / 1000000.0);


/*Torque reference*/

//  https://www.desmos.com/calculator/elzipadtpu

double offset1 = -7.5;
double lenght1 = 33;
double Amplitud1 = -1.5;  // Flexion. This one's later multiplied by K

double offset2 = 33; // earlier 30
double lenght2 = 47; //earlier 35
double Amplitud2 = 1;  // earlier 15 Extension

double final0 = offset2 + lenght2;

double Amplitud3 = -1;
double offset3 = 80;
double lenght3 = 75;

double R1 = 0;
double R2 = 0;

int LimitInf;
int LimitSup;

double test;


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
  delay(1000);
  Serial.begin(115200);//115200/9600=12
  Serial7.begin(115200);
  Serial.println("SETUP DONE");
  Serial.println("HZ:" + String(HZ / 100));
  Serial_Isra.INIT();
  //SDSetup();
  //SD_Write_begin();
  initial_CAN();
  initial_M1();
  initial_M2();
  delay(100);
  IMUSetup();
  beginning = micros();

  torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.//FOR THE TORQUE SENSORS
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 1.97, 0.0003446 * (-1) * 1.8); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.//FOR THE TORQUE SENSORS. M1 Left (tape closer to header) - The first in this list.
  torque_sensor1.Torque_sensor_offset_calibration();//FOR THE TORQUE SENSORS
  delay(1000);//FOR THE TORQUE SENSORS

  SPI1.setMOSI(26);//FOR THE TORQUE SENSORS
  SPI1.setMISO(1);//FOR THE TORQUE SENSORS
  SPI1.setSCK(27);//FOR THE TORQUE SENSORS
}

void loop() {

  imu.READ();
  t_i = micros() - beginning;
  t = t_i / 1000000.0;

  if (t_i - t_pr2 > 1400) {//Si es el CIC lo que falla, podemos disminuir esto. Era 2100
    t_pr2 = t_i;

    /*Gait*/
    cic.CIC_update(imu.LTAVx - imu.RTAVx, 0, 0, 0, 0);

  }


  if (t_i - t_pr1 > 35000) { //Si es el CIC lo que falla, podemos aumentar esto. Era 26000
    t_pr1 = t_i;

    double K = 6;
    LimitInf = -Amplitud1 * K;
    LimitSup= Amplitud1 * K;
    R1 = compute_torque(cic.GP_IMU_L);
    R2 = compute_torque(cic.GP_IMU_R);
    
    if (t_i < 5000000){
      M1_torque_command = 0;
      M2_torque_command = 0;
    }
    else{
      M1_torque_command = R1 * K;
      M2_torque_command = -R2 * K;
      //M1_torque_command =  K;
      //M2_torque_command = 0;
    }
    /*
    M1_torque_command = R1 * K;
    M2_torque_command = -R2 * K;
*/

    M1_Torque_Control_Example(); 
    Wait(1000);
    M2_Torque_Control_Example();
    Wait(1000);

    torque_sensor1.Torque_sensor_read(); //FOR THE TORQUE SENSORS


    //Wait(2000);

    print_Data();

    Transmit_ble_Data();
    //    Serial_prepare();
    //    Serial_Isra.WRITE(Send, Send_Length);
  }

  //  if (t_i - t_pr3 > 30000) {
  //    t_pr3 = t_i;
  //
  //  }

}
void print_Data() {
//Serial.print(" t ; ");
//Serial.print( t_i / 1000000 );
//Serial.print(" ");
Serial.print(M2_torque_command);
Serial.print(" ");
//Serial.print(" ; ");
Serial.print(m2.torque); //Why is the sign opposite to m1?
Serial.print(" ");
//Serial.print(" ; m2tor ; "); // Left leg is 1
Serial.print(M1_torque_command);
Serial.print(" ");
//Serial.print(" ; ");
Serial.print(m1.torque);
Serial.print(" ");
Serial.print(LimitInf);
Serial.print(" ");
Serial.print(LimitSup);
Serial.print(" ");

Serial.print(torque_sensor1.torque[0]); Serial.print("   ");//FOR THE TORQUE SENSORS
Serial.print(torque_sensor1.torque[1]); Serial.print("   ");//FOR THE TORQUE SENSORS
//Serial.print(" ; imul ; ");
//Serial.print(imu.LTx / 1);
//Serial.print(" ; ");
//Serial.print(imu.RTx / 1);
//Serial.print(" ; ");
//Serial.print(t_i); //Slow (millions of data), but required for torque tracking
//Serial.print(" ; gaitl ; ");
//Serial.print(cic.GP_IMU_L / 100);
// Serial.print(" ; m1cmd ; ");
//Serial.print(M1_torque_command);
 
//Serial.print(" ; m1tor ; "); // Left leg is 1
//Serial.print(M1_torque_command);
//Serial.print(" ; ");
//Serial.print(m1.torque);
//Serial.print(" ; imur ; ");
//Serial.print(imu.RTx / 1);
//Serial.print(" ; gaitr ; ");
//Serial.print(cic.GP_IMU_R / 100);
// Serial.print(" ; m2cmd ; ");
//Serial.print(-M2_torque_command);
//Serial.print(" ; m2tor ; ");
//Serial.print(-m2.torque);
Serial.println(" ");
}

/*
void SD_Write() {

  File dataFile = SD.open(namesd, FILE_WRITE);

  if (dataFile) {
    dataFile.print(t_i);
    dataFile.print(" ; ");
    dataFile.print(imu.LTx);
    dataFile.print(" ; ");
    dataFile.print(imu.RTx);
    dataFile.print(" ; ");
    dataFile.print(imu.LTAVx);
    dataFile.print(" ; ");
    dataFile.print(imu.RTAVx);
    dataFile.print(" ; ");
    dataFile.print(cic.GP_IMU_L);
    dataFile.print(" ; ");
    dataFile.print(cic.GP_IMU_R);
    dataFile.print(" ; ");
    dataFile.print(M1_torque_command);
    dataFile.print(" ; ");
    //    dataFile.print(m1.torque);
    //    dataFile.print(" ; ");
    dataFile.print(-M2_torque_command);
    dataFile.print(" ; ");
    //    dataFile.print(-m2.torque);
    //    dataFile.print(" ; ");
    dataFile.println("  ");
    dataFile.close();
  }

  else {
    Serial.println("error opening datalog.txt");
  }
}
*/

/*
void SD_Write_begin() {

  File dataFile = SD.open(namesd, FILE_WRITE);

  if (dataFile) {
    dataFile.print("t_i");
    dataFile.print(" ; ");
    dataFile.print("imu.LTx");
    dataFile.print(" ; ");
    dataFile.print("imu.RTx");
    dataFile.print(" ; ");
    dataFile.print("imu.LTAVx");
    dataFile.print(" ; ");
    dataFile.print("imu.RTAVx");
    dataFile.print(" ; ");
    dataFile.print("cic.GP_IMU_L");
    dataFile.print(" ; ");
    dataFile.print("cic.GP_IMU_R");
    dataFile.print(" ; ");
    dataFile.print("M1_torque_command");
    dataFile.print(" ; ");
    dataFile.print("m1.torque");
    dataFile.print(" ; ");
    dataFile.print("-M2_torque_command");
    dataFile.print(" ; ");
    dataFile.print("-m2.torque");
    dataFile.print(" ; ");
    dataFile.println("  ");
    dataFile.close();
  }

  else {
    Serial.println("error opening datalog.txt");
  }

}
*/

void M1_Position_Control_Example()
{
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0; //dont change this
  kp = 30; //max 450 min 0
  kd = 1.5; //max 5 min 0
  t_ff = 0; //dont change this
  m1.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void M2_Position_Control_Example()
{
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0; //dont change this
  kp = 30; //max 450 min 0
  kd = 1.5; //max 5 min 0
  t_ff = 0; //dont change this
  m2.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

/*void Filter_IMU() {
  f_LTAVx = LTAVx.addSample(imu.LTAVx);
  f_RTAVx = RTAVx.addSample(imu.RTAVx);
}*/

void Wait(unsigned long delay_control)
{
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  }
  while (Time_Control < Time_Delta);

}


void IMUSetup()
{
  imu.INIT();
  delay(500);
  imu.INIT_MEAN();
}

void M1_Torque_Control_Example()
{
  p_des = 0; //dont change this
  v_des = 0; //dont change this
  kp = 0; //dont change this
  kd = 0; //dont change this
  t_ff = M1_torque_command;
  m1.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void M2_Torque_Control_Example()
{
  p_des = 0; //dont change this
  v_des = 0; //dont change this
  kp = 0; //dont change this
  kd = 0; //dont change this
  t_ff = M2_torque_command;
  m2.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}

void initial_M1()
{
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

void initial_M2()
{
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

void receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    int id = msgR.buf[0];
    //Serial.print(msgR.id, HEX );
    if (id == Motor_ID)
    {
      m1.unpack_reply(msgR);
    }
    if (id == Motor_ID2)
    {
      m2.unpack_reply(msgR);
    }
  }
}

double compute_torque(double g) {
  double R = 0;
  if (g < offset1) {
    R = 0;
  } else if (offset1 < g && g <= lenght1) {
    R = Amplitud1 * sin((g - offset1) * (3.1416 / (lenght1 - offset1)));
  } else if (lenght1 < g && g <= offset2) {
    R = 0;
  } else if (offset2 < g && g <= final0) {
    R = Amplitud2 * sin((g - offset2) * 3.1416 / lenght2);
  } else if (final0 < g && g < 100) {
    R = Amplitud3 * sin((g - offset3) * 3.1416 / lenght3);;
  }
  return R;
}


/*
void SDSetup() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
    }
  }
  Serial.println("card initialized.");

  if (SD.exists(namesd)) {
    SD.remove(namesd);
    Serial.println(" removed ");
  }
  Serial.println(String(namesd) + " Done");
}*/

void Serial_prepare() {


  L_IMUX_int = Serial_Isra.float_to_uint(imu.LTx, -180, 180, 16);
  R_IMUX_int = Serial_Isra.float_to_uint(imu.RTx, -180, 180, 16);

  L_IMUV_int = Serial_Isra.float_to_uint(imu.LTAVx, -800, 800, 16);
  R_IMUV_int = Serial_Isra.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_tor_int = Serial_Isra.float_to_uint(m1.torque, -25, 25, 16);
  R_tor_int = Serial_Isra.float_to_uint(-m2.torque, -25, 25, 16);

  L_cmd_int = Serial_Isra.float_to_uint(M1_torque_command, -25, 25, 16);
  R_cmd_int = Serial_Isra.float_to_uint(-M2_torque_command, -25, 25, 16);

  GP_IMU_L_int = Serial_Isra.float_to_uint(cic.GP_IMU_L, -10, 110, 16);
  GP_IMU_R_int = Serial_Isra.float_to_uint(cic.GP_IMU_R, -10, 110, 16);

  Send[0] = 0x31; Send[1] = 0x32;

  Send[2] = L_IMUX_int >> 8; Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8; Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8; Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8; Send[9] = R_IMUV_int & 0xFF;
  Send[10] = L_tor_int >> 8; Send[11] = L_tor_int & 0xFF;
  Send[12] = R_tor_int >> 8; Send[13] = R_tor_int & 0xFF;
  Send[14] = L_cmd_int >> 8; Send[15] = L_cmd_int & 0xFF;
  Send[16] = R_cmd_int >> 8; Send[17] = R_cmd_int & 0xFF;
  Send[18] = GP_IMU_L_int >> 8; Send[19] = GP_IMU_L_int & 0xFF;
  Send[20] = GP_IMU_R_int >> 8; Send[21] = GP_IMU_R_int & 0xFF;

  Send[22] = 0x32; Send[23] = 0x32;
  Send[24] = 0x32; Send[25] = 0x32;
  Send[26] = 0x32; Send[27] = 0x32;
  Send[28] = 0x32; Send[29] = 0x33;

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