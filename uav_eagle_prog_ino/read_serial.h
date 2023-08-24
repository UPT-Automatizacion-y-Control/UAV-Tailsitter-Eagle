#include "esp32-hal.h"
float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
char read_byte()
{
  while(!Serial2.available());
  
  return Serial2.read();
}
bool read_serial () {   
  
  psi = read_byte()-1;
  if (psi == 254)   return true;

  z_p = read_byte()-1;
  if (z_p == 254)   return true;

  theta = read_byte()-1;
  if (theta == 254) return true;

  phi = read_byte()-1;
  if (phi == 254)  return true; 

  buttons = read_byte()-1;  
  if (buttons == 254) return true;
 
  phi_ang = fmap(phi, 0, 250, -5, 5);
  theta_g =  90.0 + fmap(theta,  0, 250,   10,   -10);
  
  return false;
}

void polling_serial2(){
  
    if (Serial2.available()){
    if (Serial2.find(255)){
      if (read_serial()) 
        read_serial();
    }
  }
}