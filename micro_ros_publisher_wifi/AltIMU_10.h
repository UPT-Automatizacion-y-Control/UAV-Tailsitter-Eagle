
// Magneto calibrado 05/06/23
float mag_hiron_offsets[3] = { -69.88, -64.14, 1.70 };
float mag_siron_offsets[3][3] = { { 0.983, 0.020, -0.010 },
                                  { 0.020, 0.995,  0.005 },
                                  {-0.010, 0.005,  1.022 } };
// Acelerómetro calibrado 14/05/23
float acc_bias[3] = {0.0262, 0.031824, -0.002348};
float acc_sf_matrix[3][3] = {{ 1.007368, 0.000527, -0.004488},
                             { 0.000527, 1.006978,  0.001624},
                             {-0.004488, 0.001624,  0.995385}};

float gyro_bias[3] = {0.050559,-0.098304, -0.082233};      
                       
void mag_read(float &magnetom_x, float &magnetom_y, float &magnetom_z) {
  mag.read();
  // Lectura en uT
  float mag_xyz_hiron[3] = { (float)mag.m.x * Gauss_to_uT,    // Lectura de mag_x
                             (float)mag.m.y * Gauss_to_uT,    // Lectura de mag_y
                             (float)mag.m.z * Gauss_to_uT };  // Lectura de mag_z
  // Corrección de mag de valores de Hard Iron
  for (int i = 0; i < 3; i++) {
    mag_xyz_hiron[i] = mag_xyz_hiron[i] - mag_hiron_offsets[i];
  }
  // Corrección de mag de valores de Soft Iron
  float mag_data_calib[3];
  for (uint8_t i = 0; i < 3; i++) {
    mag_data_calib[i] = (mag_siron_offsets[i][0] * mag_xyz_hiron[0]) + (mag_siron_offsets[i][1] * mag_xyz_hiron[1]) + (mag_siron_offsets[i][2] * mag_xyz_hiron[2]);
  }
  // Entrega de datos de Magnetómetro corregidos
  magnetom_x = mag_data_calib[0];
  magnetom_y = mag_data_calib[1];
  magnetom_z = mag_data_calib[2];
}
void gyro_accel_read(float &acc_x, float &acc_y, float &acc_z, float &gyro_x, float &gyro_y, float &gyro_z) {
  
  gyro_acc.readAcc();
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
  acc_x = acc_cal_data[0] * 9.80665F;
  acc_y = acc_cal_data[1] * 9.80665F;
  acc_z = acc_cal_data[2] * 9.80665F;

  // Corrección de datos de giroscopio
  gyro_acc.readGyro();
  gyro_x = dps_to_rds(gyro_acc.g.x) - gyro_bias[0];
  gyro_y = dps_to_rds(gyro_acc.g.y) - gyro_bias[1];
  gyro_z = dps_to_rds(gyro_acc.g.z) - gyro_bias[2];
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

  mag.init();

  mag.enableDefault();

  mag.writeReg(LIS3MDL::CTRL_REG1, 0b11111100);  // 155hz rate
  mag.writeReg(LIS3MDL::CTRL_REG2, 0b01100000);  // +/- 8 gauss
  mag.writeReg(LIS3MDL::CTRL_REG3, 0X00);  // Continuous conversion mode enable
  mag.writeReg(LIS3MDL::CTRL_REG4, 0X0C);  // UH performance mode in z axis
  
}
void Update_angles(euler &ypr_angles) {
  
  gyro_accel_read(ax, ay, az, gx, gy, gz);
  mag_read(mx, my, mz);

   deltat = fusion.deltatUpdate();
  if (!accel_flag) {
    accel_flag = true;
    t_angles = millis();
  }

  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);
  
  ypr_angles.roll   = roll_f.updateEstimate(fusion.getPitch() + 1.0);
  ypr_angles.pitch  = pitch_f.updateEstimate(179 - fusion.getRoll());
  ypr_angles.yaw    = fusion.getYaw();
  
}
