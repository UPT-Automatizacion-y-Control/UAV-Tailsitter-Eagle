float error[4], prev_error[4], intl[4], derv, p, i, d;
float pa_p[4];
unsigned long p_dt;
int16_t s ;
bool stop_flag = true, integrator_stop = true;

void saturacion_motor(motor &mtr, int16_t min, int16_t max){
    if (mtr.D < min)
    mtr.D = min;
  else if (mtr.D > max)
    mtr.D = max;

    if (mtr.I < min)
    mtr.I = min;
  else if (mtr.I > max)
    mtr.I = max;
}
void saturacion(float &valor, int16_t min, int16_t max){
  if (valor < min)
    valor = min;
  else if (valor > max)
    valor = max;
}
void PID_compute(float pa, float setpoint, k_pid k, uint8_t ypr, float &ctrl_s, int16_t min, int16_t max, int16_t i_min, int16_t i_max,float dt){
    
    float dt_c = dt/1000.0;
    error[ypr] = setpoint - pa;
    
    if (ypr == 0){
      if (error[ypr] > 180)
        error[ypr] -= 360;
      else if (error[ypr] < -180)
        error[ypr] += 360;
    }

    // Proporcional
    p = error[ypr] * k.p;

    // integral
    if (!integrator_stop)
      intl[ypr] += error[ypr] * dt_c;
    else
      intl[ypr] = 0;

    i = intl[ypr] * k.i;

    if (i < i_min) {
      i = i_min;
      intl[ypr] = i/k.i;
      }
    else if(i > i_max) {
      i = i_max; 
      intl[ypr] = i/k.i;
    }
    

    // Derivativo
    derv = (pa - pa_p[ypr]) / dt_c;
    pa_p[ypr] = pa;
    d = -derv * k.d;



    s = p + i + d;

    if (s < min)
      s = min;
    else if (s > max)
      s = max;
      
    ctrl_s = s;

  return;
}