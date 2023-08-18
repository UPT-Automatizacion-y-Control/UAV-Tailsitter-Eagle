#include <LSM6.h>
#include <LIS3MDL.h>
#include <Servo.h>
#include "HardwareSerial.h"
Servo ESC1, ESC2, SERV1, SERV2;
#define SDA_I2C_PIN 33
#define SCL_I2C_PIN 32
#define ToRad(x) ((x)*0.01745329252)
#define Gauss_to_uT 100.0f / 1711.0f
#define g_LSB 0.488f / 1000.0f // 2g = 0.061; 4g = 0.122; 8g = 0.244; 16g = 0.488
#define mg_LSB_to_ms2(x) (x) * g_LSB * 9.80665
#define dps_to_rds(x) ((x)*ToRad(0.070))

double dps_to_rads = 0.00122173;
double mg_to_ms2 = g_LSB*9.80665;
LSM6 gyro_acc;
LIS3MDL mag;
void UAV_init() 
{
  // Pines para control de ESC
  ESC1.attach(25); 
  ESC2.attach(26);
  // Pines para control de Servos
  SERV1.attach(27);
  SERV2.attach(14);
  // Señal de inicio para configuración de velocidad mínima en los ESC
  ESC1.writeMicroseconds(900);
  ESC2.writeMicroseconds(900);
  delay(3000);
}
void signal_motors(float ESC1_M, float ESC2_M, float SVO1_M, float SVO2_M){
  ESC1.writeMicroseconds(ESC1_M);
  ESC2.writeMicroseconds(ESC2_M); 
  SERV1.write(SVO1_M);
  SERV2.write(SVO2_M);
}
void IMU_Init() {
  Wire.begin(SDA_I2C_PIN, SCL_I2C_PIN);
 
  gyro_acc.init();
  
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL,0b00110111); // 52 Hz, 16 g full scale
  gyro_acc.writeReg(LSM6::CTRL2_G, 0b00111100); // 52 Hz, 2000 dps full scale
  gyro_acc.writeReg(LSM6::CTRL8_XL,0b10000100); // Enable LP filter set at ODR/400
  gyro_acc.writeReg(LSM6::CTRL7_G, 0b00000000); 
  gyro_acc.writeReg(LSM6::TAP_CFG, 0x10); 
  
  // Enable accel functions
  //gyro_acc.writeReg(LSM6::TAP_CFG, 0x10);
  mag.init();

  mag.enableDefault();

  mag.writeReg(LIS3MDL::CTRL_REG1, 0b11111100);  // 80Hz rate
  mag.writeReg(LIS3MDL::CTRL_REG2, 0b01100000);  // +/- 16 gauss
  mag.writeReg(LIS3MDL::CTRL_REG3, 0X00);  // Continuous conversion mode enable
  mag.writeReg(LIS3MDL::CTRL_REG4, 0X0C);  // UH performance mode in z axis
}
void setup() {
  Serial.begin(115200);
 
  IMU_Init();
  UAV_init();
}

void loop() {
  signal_motors(1300 , 1300, 90, 85);
  gyro_acc.readAcc();
  gyro_acc.readGyro();
  mag.read();
  int acc[3] = {  gyro_acc.a.x, 
                  gyro_acc.a.y,
                  gyro_acc.a.z};
  int gyr[3] = {  gyro_acc.g.x, 
                  gyro_acc.g.y,
                  gyro_acc.g.z};
  int magn[3] = { mag.m.x,
                  mag.m.y,
                  mag.m.z};

  Serial.print("Raw:");
  Serial.print(acc[0]);Serial.print(",");
  Serial.print(acc[1]);Serial.print(",");
  Serial.print(acc[2]);Serial.print(",");
  Serial.print(gyr[0]);Serial.print(",");
  Serial.print(gyr[1]);Serial.print(",");
  Serial.print(gyr[2]);Serial.print(",");
  Serial.print((int)(magn[0]*Gauss_to_uT*10));Serial.print(",");
  Serial.print((int)(magn[1]*Gauss_to_uT*10));Serial.print(",");
  Serial.print((int)(magn[2]*Gauss_to_uT*10));Serial.println("");

  Serial.print("Uni:");
  Serial.print(mg_to_ms2*(acc[0]));Serial.print(",");
  Serial.print(mg_to_ms2*(acc[1]));Serial.print(",");
  Serial.print(mg_to_ms2*(acc[2]));Serial.print(",");
  Serial.print(dps_to_rads*gyr[0],4);Serial.print(",");
  Serial.print(dps_to_rads*gyr[1],4);Serial.print(",");
  Serial.print(dps_to_rads*gyr[2],4);Serial.print(",");
  Serial.print(magn[0]*Gauss_to_uT);Serial.print(",");
  Serial.print(magn[1]*Gauss_to_uT);Serial.print(",");
  Serial.print(magn[2]*Gauss_to_uT);Serial.println("");
  Serial.println(g_LSB,8);
  delay(20);
}
