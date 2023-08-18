
#include <Adafruit_SensorLab.h>
#include <Adafruit_Sensor_Calibration.h>

#define SDA_I2C_PIN 33
#define SCL_I2C_PIN 32
#include <Servo.h>
Adafruit_SensorLab lab;

Servo ESC1, ESC2, SERV1, SERV2;
int t[2];
Adafruit_Sensor *mag = NULL, *gyro = NULL, *accel = NULL;
sensors_event_t mag_event, gyro_event, accel_event;

int loopcount = 0;
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
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  UAV_init();
  Serial.println(F("Sensor Lab - IMU Calibration!"));
  Wire.begin(SDA_I2C_PIN, SCL_I2C_PIN);
  lab.begin();
  Serial.println("Looking for a magnetometer");
  mag = lab.getMagnetometer();
  if (! mag) {
    Serial.println(F("Could not find a magnetometer, skipping!"));
  } else {
    mag->printSensorDetails();
  }
  
  Serial.println("Looking for a gyroscope");
  gyro = lab.getGyroscope();
  if (! gyro) {
    Serial.println(F("Could not find a gyroscope, skipping!"));
  } else {
    gyro->printSensorDetails();
  }
  
  Serial.println("Looking for a accelerometer");
  accel = lab.getAccelerometer();
  if (! accel) {
    Serial.println(F("Could not find a accelerometer, skipping!"));
  } else {
    accel->printSensorDetails();
  }
}
void signal_motors(float ESC1_M, float ESC2_M, float SVO1_M, float SVO2_M){
  ESC1.writeMicroseconds(ESC1_M);
  ESC2.writeMicroseconds(ESC2_M); 
  SERV1.write(SVO1_M);
  SERV2.write(SVO2_M);
}

void loop() {
  t[0] = millis() - t[1];
  t[1] = millis();
  signal_motors(1200, 1200, 85, 90);
  if (mag && ! mag->getEvent(&mag_event)) {
    return;
  }
  if (gyro && ! gyro->getEvent(&gyro_event)) {
    return;
  }
  if (accel && ! accel->getEvent(&accel_event)) {
    return;
  }
  // 'Raw' values to match expectation of MOtionCal
  Serial.print("Raw:");
  Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.x*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.y*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.z*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");
  
  // unified data
  Serial.print("Uni:");
  Serial.print(accel_event.acceleration.x); Serial.print(",");
  Serial.print(accel_event.acceleration.y); Serial.print(",");
  Serial.print(accel_event.acceleration.z); Serial.print(",");
  Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
  Serial.print(mag_event.magnetic.x); Serial.print(",");
  Serial.print(mag_event.magnetic.y); Serial.print(",");
  Serial.print(mag_event.magnetic.z); Serial.println("");
  
}
