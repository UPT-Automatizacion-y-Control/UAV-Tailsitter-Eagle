#include <micro_ros_arduino.h> 
#include <stdio.h>
#include "Adafruit_AHRS_NXPFusion.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include "SensorFusion.h"
#include <Servo.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <VL53L1X.h>
#include <Wire.h>
#include "HardwareSerial.h"
#include "clases.h"
#include "Control.h"
#include <SimpleKalmanFilter.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define Gauss_to_uT 100.0f / 1711.0f
#define dps_to_rds(x) ((x)*ToRad(0.070))
#define g_LSB 0.488f / 1000.0f // 2g = 0.061; 4g = 0.122; 8g = 0.244; 16g = 0.488
#define mg_LSB_to_ms2(x) (x) * g_LSB * 9.80665
#define pi 3.14159265
#define V_LOW 900
#define V_HIGH 2100
#define SDA_I2C_PIN 33
#define SCL_I2C_PIN 32

//#define betaDef 0.2f

SF fusion;
Adafruit_NXPSensorFusion filter;
VL53L1X VL1;
LSM6 gyro_acc;
LIS3MDL mag;
SimpleKalmanFilter skf(1, 1, 0.1);
SimpleKalmanFilter pitch_f(0.3, 0.3, 0.8);
SimpleKalmanFilter roll_f(0.2, 0.2, 0.9);
unsigned long int time_rob = 0;
unsigned int  buttons,  dt_z = 0, time_p_z = 0, t_z = 0, t, vel_min,  contador = 0, t_angles = 0, t_ud_angles = 0, mag_freq[2];
int z_ref;
char mag_counter = 0;
signed char psi_20_inc_dec = 0;
float derivada, integral;
float psi_ang, theta_g = 0.0, phi_ang, psi_ang_0_360, z_p, phi, psi, theta;
float ctrl_z = 0, ctrl_SVO = 0, ctrl_ESC, ctrl_yaw, z;
float theta_ang[2], angulo_reposo = 0, z_pas, z_r, alpha, robot_yaw;
float gx, gy, gz, ax, ay, az, mx, my, mz, pitch, roll, yaw, deltat;
bool flag_power, flag_c = true, accel_flag = false, flag_turn = true;
float s_ctrl[3];
euler UAV_angles;
Servo ESC1, ESC2, SERV1, SERV2;
motor ESC, Servo;

/*      GANANCIAS DE LOS CONTROLADORES    */
k_pid k_roll (03.50, 01.00, 01.35); // 4.6 2 1.35
k_pid k_pitch(00.80, 00.72, 00.20); // 0.8 0.65 0.16
k_pid k_yaw  (00.95, 00.00, 00.20); 
k_pid k_z    (00.082, 00.016, 00.060); //(0, 0 ,0 );//(00.082, 00.016, 00.060); //(0, 0 ,0 );//

#include "read_serial.h"
#include "Motors.h"
#include "uRos.h"
#include "AltIMU_10.h"
#include "VL53L1X_Conf.h"

void setup() {
  // Inicialización del Serial ...
  //Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  // Inicialización de la IMU
  
  IMU_Init();
  
  // Inicialización del sensor  ToF de distancia
  VL53L1X_Init();
  
  // Configuración de la Red WiFi para uRos
  //Serial.print("conectando a la red ...");
  set_microros_wifi_transports((char *)"kikat", (char *)"290220KyK1", (char *)"10.42.0.1", 8888);
  
  //set_microros_wifi_transports((char *)"LabExp_AutomatyControl", (char *)"84896209", (char *)"192.168.0.107", 8888); 
  // Configuración de los ESC's
  UAV_init();
  // Configuración y creación del nodo de uRos
  uRos_Conf();
  
}

void loop() {

  polling_serial2();


if (millis()-time_rob >= 19){
    
    t = millis()-time_rob;
    time_rob = millis();
    Update_angles(UAV_angles);
    robot_yaw = fmap(float(psi), 0.0f, 250.0f, -1.0f, 1.0f);

     if (buttons & 0b01000000 && flag_turn){
        psi_20_inc_dec = -1;
        flag_turn = false;
      }
    if (buttons & 0b10000000 && flag_turn){
        psi_20_inc_dec = 1;
        flag_turn = false;
        }
      if (!(buttons & 0b10000000) &&  !(buttons & 0b01000000) && !flag_turn)
        flag_turn = true;
   
    psi_ang += robot_yaw * 0.72 + psi_20_inc_dec*20;

    psi_20_inc_dec = 0;

    psi_ang_0_360 = fmod(psi_ang, 360.0);

    if (psi_ang_0_360 < 0)
      psi_ang_0_360 += 360;

    contador++;
    if (contador == 2) {
      contador = 0;
      time_p_z = millis();

      z = VL1.read(false);

      z = skf.updateEstimate(z);
      
      // Cómputo de la distancia en z de acuerdo a la orientación del UAV
      pitch = ToRad(UAV_angles.pitch);
      roll = ToRad(UAV_angles.roll);
      z = z * cos(pitch - pi / 2) * cos(roll);

      if (buttons & 0b00010000)
        z_ref += 15;
      else if (buttons & 0b00100000)
        z_ref -= 15;

      if (z_ref > 2000) 
        z_ref = 2000;
      else if (z_ref < 0) 
        z_ref = 0;
      else if (buttons & 0b00000100 || z_p == 0)
        z_ref = 0;
      else if (z_ref == 0 && z_p == 250)
        z_ref = 1000;

     

      dt_z = millis() - t_z;

      PID_compute(z, z_ref, k_z, 3, ctrl_z, -300, 300, -200, 200, dt_z);
      t_z = millis();

      attitude.angular.x = UAV_angles.roll;
      attitude.angular.y = UAV_angles.pitch;
      attitude.angular.z = UAV_angles.yaw;
      attitude.linear.z = z;
      attitude.linear.x = s_ctrl[0];
      attitude.linear.y = s_ctrl[1];

      rcl_publish(&publisher_attitude, &attitude, NULL);

      attitude_references.angular.x = phi_ang;
      attitude_references.angular.y = theta_g;
      attitude_references.angular.z = psi_ang_0_360;
      attitude_references.linear.z = z_ref;
      attitude_references.linear.y = ESC.D;
      attitude_references.linear.x = s_ctrl[2];
      rcl_publish(&publisher_attitude_references, &attitude_references,NULL);
    }

    PID_compute(UAV_angles.pitch, theta_g, k_pitch, 1, ctrl_SVO, -80, 80, -30, 30, t);
    PID_compute(UAV_angles.roll, phi_ang, k_roll, 2, ctrl_ESC, -100, 100, -50, 50, t);
    s_ctrl[0] = p;
    s_ctrl[1] = i;
    s_ctrl[2] = d;
    PID_compute(UAV_angles.yaw, psi_ang_0_360, k_yaw, 0, ctrl_yaw, -80, 80, -20, 20, t);
    /*--------------------------------------------------------------------------*/
    // Parámetros de calibración del sistema de 4 barras
    theta_ang[0] =  20 - ctrl_SVO;
    theta_ang[1] = 160 + ctrl_SVO; // 20
    
    saturacion(theta_ang[0], -60, 80); // 110 220
    saturacion(theta_ang[1], 130, 360);

    Servo.D = theta_ang[1] - ctrl_yaw;
    Servo.I = theta_ang[0] - ctrl_yaw;

    // Transformación del ángulo del sistema de 4 barras, el segundo parámetro indica sentido del sistema 4_b
    Servo_ang(Servo.D, 1);
    Servo_ang(Servo.I, 0);
    
    // Parámetros de calibración de las siperficies de control
    Servo.D -= 20; //
    Servo.I -= 0;
    
    if (buttons & 0b00001000){
      flag_power = false;
      stop_flag = false;
      integrator_stop = true;
    }

    if (buttons & 0b00000001 && !flag_power) {
      integrator_stop = false;
      flag_power = true;
      psi_ang = UAV_angles.yaw;
    } 
    if (!flag_power)
      psi_ang = UAV_angles.yaw;

    else if (buttons & 0b00000010){ 
      stop_flag = true;
      integrator_stop = true;
     }

    flag_c = true;

    if (ctrl_ESC > 0)
      flag_c = false;

    if (z_ref == 0 && z <= 200 || !flag_power)
      vel_min = 1400;
    else
      vel_min = 1635;

    
   // vel_min = 1500;
    ESC.I = vel_min + (ctrl_z - ctrl_ESC)*flag_power;
    ESC.D = vel_min + (ctrl_z + ctrl_ESC)*flag_power;

    saturacion_motor(ESC, 1050, 1850);
    saturacion_motor(Servo, 0, 180);

    if (UAV_angles.pitch > 170.0 || UAV_angles.pitch < 0.0 || UAV_angles.roll > 70.0 || UAV_angles.roll < -70.0 || stop_flag || millis() - t_angles < 8000) {
      ESC.D = V_LOW;
      ESC.I = V_LOW;
      Servo.D = 90;
      Servo.I = 70;
      stop_flag = true;
      flag_power = false;
      
    }
    
    signal_motors(ESC.D, ESC.I, Servo.D, Servo.I);
  }
}
