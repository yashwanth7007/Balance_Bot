class PID{
 


public:
 float Kp,Ki,Kd,value;
 float kp_bridge,ki_bridge,kd_bridge,kp_normal,ki_normal,kd_normal;
 float out;
 
 float error=0.0,error_integral=0.0,Past_error=0.0,derivative;

 
   PID(float kp,float ki,float kd,float kp_B,float ki_B,float kd_B){
    Kp=kp;
    Ki=ki;
    Kd=kd;
    kp_bridge=kp_B;
    ki_bridge=ki_B;
    kd_bridge=kd_B;
    kp_normal=kp;
    ki_normal=ki;  
    kd_normal=kd;
  }

  
 
  void PID_controller(float set_point,float reading,float dT){
   error =  set_point - reading;
   derivative = (error - Past_error)/dT;
   error_integral =  error_integral + (error * dT);
   error_integral= constrain(error_integral,-value,value);
   out = (Kp*error) + (Kd*derivative) + (Ki*error_integral);
  out=constrain(out,-value,value);
  Past_error = error;
  
}
void reset(){
  error=0 ;
  Past_error=0;
  error_integral=0 ;
  derivative=0 ;
  
}

void limit(float input){
 value = input;
}


void change_constants(int mode){
	if(!mode){
  Kp = kp_bridge;
  Ki = ki_bridge;
  Kd = kd_bridge;
}
else{
  Kp = kp_normal;
  Ki = ki_normal;
  Kd = kd_bridge;
}
}

};

class Low_pass{
  public:
  float out;

  low_pass();

  void filter(float present,float alpha){
    out = alpha*out + (1-alpha)*present;
  }

  
};
