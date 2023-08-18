#include <stdio.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define Gauss_to_uT 100.0f / 3421.0f
#define dps_to_rds(x) ((x)*ToRad(0.07))
#define g_LSB 0.488f / 1000.0f
#define mg_LSB_to_ms2(x) (x) * g_LSB * 10.0
#define pi 3.14159265
#define V_LOW 900
#define V_HIGH 2100
#define SDA_I2C_PIN 33
#define SCL_I2C_PIN 32

LSM6 gyro_acc;
LIS3MDL mag;
int t_gyr;
String gyr_axis[3] = {"gx", "gy", "gz"};
bool gyr_meas = true;
unsigned int time_delay = 20, t;
float ax,ay,az;
int contador;
float gyr_data[3];
float acc_bias[3] = {0.0262, 0.031824, -0.002348};
float acc_sf_matrix[3][3] = {{ 1.007368, 0.000527, -0.004488},
                             { 0.000527, 1.006978,  0.001624},
                             {-0.004488, 0.001624,  0.995385}};

void gyro_accel_read(float &acc_x, float &acc_y, float &acc_z, float &gyro_x, float &gyro_y, float &gyro_z) {
  
  gyro_acc.readAcc();
  float acc_G[3] = {gyro_acc.a.x * g_LSB,
                    gyro_acc.a.y * g_LSB,
                    gyro_acc.a.z * g_LSB};
  float acc_bias_correction[3];
  float acc_cal_data[3];

  // Correción de offsets
  for (int i = 0; i<3;i++)
    acc_bias_correction[i] = acc_G[i] - acc_bias[i];

  for (int i = 0; i<3; i++){
    acc_cal_data[i] = acc_sf_matrix[i][0] * acc_bias_correction[0] +
                      acc_sf_matrix[i][1] * acc_bias_correction[1] +
                      acc_sf_matrix[i][2] * acc_bias_correction[2];
  }
   

  // Conversión de datos de g a ms2
  acc_x = acc_cal_data[0] * 9.80665;
  acc_y = acc_cal_data[1] * 9.80665;
  acc_z = acc_cal_data[2] * 9.80665;
  
}

void IMU_Init() {

  Wire.begin(SDA_I2C_PIN, SCL_I2C_PIN);

  gyro_acc.init();

  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL,0b01000111); // 104 Hz, 16 g full scale
  gyro_acc.writeReg(LSM6::CTRL2_G, 0b01001100); //104 Hz, 2000 dps full scale
  gyro_acc.writeReg(LSM6::CTRL8_XL,0b10000100);  // Enable LP filter set at ODR/400
  gyro_acc.writeReg(LSM6::CTRL7_G, 0b00111000); 
  gyro_acc.writeReg(LSM6::TAP_CFG, 0x10); 
  
}
void setup() {
  // Inicialización del Serial ...
  Serial.begin(115200);
  IMU_Init();
  delay(5000);
  t_gyr = millis();
}

void loop() {

  if (t+time_delay < millis()){
    t = millis();
  // Descomentar la siguientes líneas para envíar la información al programa
  // en C++ para guardar los datos calculados.
    /*gyro_acc.readAcc();
    ax = 0.488F*(float)gyro_acc.a.x/1000.0F;
    ay = 0.488F*(float)gyro_acc.a.y/1000.0F;
    az = 0.488F*(float)gyro_acc.a.z/1000.0F;
    float magnitude = std::sqrt(pow(ax,2)+ pow(ay,2)+ pow(az,2));

    Serial.print("s");
    Serial.print(ax,6);
    Serial.print(",");
    Serial.print(ay,6);
    Serial.print(",");
    Serial.println(az,6);
    /*Serial.print(",");
    Serial.println(magnitude,6);*/

  // Descomentar las líneas de abajo para realizar la calibración del giroscopio
  gyro_acc.readGyro();
   double gyr_[3] = {((double)gyro_acc.g.x)*ToRad(0.070),
                     ((double)gyro_acc.g.y)*ToRad(0.070),
                     ((double)gyro_acc.g.z)*ToRad(0.070)};
  
 /* Serial.print(gyr_[0],4);
  Serial.print(" ");
  Serial.print(gyr_[1],4);
  Serial.print(" ");
  Serial.println(gyr_[2],4);*/
  //if (t_gyr + 2000 < millis()){
 
    if (true) {
  if (gyr_meas){
    Serial.print(gyr_[0],4);
    Serial.print(" ");
    Serial.print(gyr_[1],4);
    Serial.print(" ");
    Serial.println(gyr_[2],4);

  
    gyr_data[0] += gyr_[0];
    gyr_data[1] += gyr_[1];
    gyr_data[2] += gyr_[2];
    contador ++;
  }
  
  if (contador == 200){
    contador ++;
    gyr_meas = false;
    for(int a = 0; a < 3; a++){
      gyr_data[a] = gyr_data[a]/contador;
    }
    Serial.println("Offsets calculados del giroscopio ...");

    for(int a = 0; a < 3; a++){
      Serial.print(gyr_axis[a]);
      Serial.print(": ");
      Serial.print(gyr_data[a],6);
      Serial.print(" ");
  }
  }
  }
  // Descomentar las siguientes líneas para monitorear los datos del acelerómetro
  // ya calculados con la corrección de los offsets y la escala del vector. 
 /* gyro_acc.readAcc();
  float acc_G[3] = {(float)gyro_acc.a.x * g_LSB,
                    (float)gyro_acc.a.y * g_LSB,
                    (float)gyro_acc.a.z * g_LSB};
  float acc_bias_correction[3];
  float acc_cal_data[3];

  // Correción de offsets
  for (int i = 0; i<3;i++)
    acc_bias_correction[i] = acc_G[i] - acc_bias[i];

  for (int i = 0; i<3; i++)
    acc_cal_data[i] = acc_sf_matrix[i][0] * acc_bias_correction[0] +
                      acc_sf_matrix[i][1] * acc_bias_correction[1] +
                      acc_sf_matrix[i][2] * acc_bias_correction[2];

  // Conversión de datos de g a ms2
  float acc_x = acc_cal_data[0] * 9.80665;
  float acc_y = acc_cal_data[1] * 9.80665;
  float acc_z = acc_cal_data[2] * 9.80665;

  Serial.print(acc_x, 4);
  Serial.print(" ");
  Serial.print(acc_y, 4);
  Serial.print(" ");
  Serial.println(acc_z, 4);*/
  
    //delay(20);
  } 
}