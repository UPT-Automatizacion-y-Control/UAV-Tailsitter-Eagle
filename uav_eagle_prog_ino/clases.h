class euler{
  public:
    float yaw;
    float pitch;
    float roll;
    euler(float y=0, float p=0, float r=0){
      yaw = y;
      pitch = p;
      roll = r;
    }
};

class k_pid {
  public:
    float p;
    float i;
    float d;
    k_pid(float p_, float i_, float d_){
      p = p_;
      i = i_;
      d = d_;
    }
};

class motor {
  public:
    float I;
    float D;
};