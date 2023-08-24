void UAV_init() 
{
  // Pines para control de ESC
  ESC1.attach(25); 
  ESC2.attach(26);
  // Pines para control de Servos
  SERV1.attach(27);
  SERV2.attach(14);
  // Señal de inicio para configuración de velocidad mínima en los ESC
  ESC1.writeMicroseconds(V_LOW);
  ESC2.writeMicroseconds(V_LOW);
  delay(3000);
}
void signal_motors(float ESC1_M, float ESC2_M, float SVO1_M, float SVO2_M){
  ESC1.writeMicroseconds(ESC1_M);
  ESC2.writeMicroseconds(ESC2_M); 
  SERV1.write(SVO1_M);
  SERV2.write(SVO2_M);
}
/* -----      -----     -----     ------      ------      ------      ----*/
/*                      SISTEMA DE 4 BARRAS DE ALERONES                 */

const float l[4] = {16.65, 35, 45, 14.29 };
const double theta_f_bar[2] = {3.7563,-0.614704963 };
const double l_c_0[2] = {l[0]*cos(theta_f_bar[0]), l[0]*cos(theta_f_bar[1])};
const double l_s_0[2] = {l[0]*sin(theta_f_bar[0]), l[0]*sin(theta_f_bar[1])};
const float l_3_pow_2 = pow(l[3],2);
const float l_2_pow_2 = pow(l[2],2);
const float d_l_2 = 2*l[2], d_l_3 = 2*l[3];
unsigned int cont = 0;

void Servo_ang (float &theta_b, bool side) {
  float A, B, C, D;
  theta_b = ToRad(theta_b);
  
  if (side){
    A = l_c_0[0] + l[1]*cos(theta_b);
    B = l_s_0[0] + l[1]*sin(theta_b);
    C = (l_3_pow_2 - l_2_pow_2 - pow(A,2) - pow(B,2))/(d_l_2);
    D = (l_2_pow_2 - l_3_pow_2 - pow(A,2) - pow(B,2))/(d_l_3);

    theta_b = ToDeg(2*atan2(B+sqrt(pow(A,2)+pow(B,2)-pow(D,2)),D + A));
    theta_b = theta_b - 180.0; // Transformación del ángulo. 
  }
  else {
    A = l_c_0[1] + l[1]*cos(theta_b);
    B = l_s_0[1] + l[1]*sin(theta_b);
    C = (l_3_pow_2 - l_2_pow_2 - pow(A,2) - pow(B,2))/(d_l_2);
    D = (l_2_pow_2 - l_3_pow_2 - pow(A,2) - pow(B,2))/(d_l_3);
    theta_b = ToDeg(2*atan2(B-sqrt(pow(A,2)+pow(B,2)-pow(D,2)),D + A));
    theta_b = 180 + theta_b; // Transformación del ángulo. 
  }
}