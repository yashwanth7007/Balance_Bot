 #include "pid.h"
#include <Wire.h>
#include "Motor.h"
#include "MPU.h"
#include "Remote.h"
#include "Indicators.h"
#include "timer.h"
#define PITCH_ANGLE 2
#define ERROR_threshold 100

int l;

bool rotation_flag= false;
bool bridge_flag=false;

int sw=0;
int tp,t1;

PID phi_set(0.0005,0,0,0.004,0,0),phidot_set(0.015,0.01,0.0001,0.01,0,0),theta(15,1,0.002,13,13,0),steer(0.1,0,0,0.1,0,0); //11.6 has good rejection for ground
Low_pass phidot1,phidot2;

float pitch,roll;
int x,y,MagF,MagB,mode,slope,present_val=0,previous_val=0;
float phi_setpoint,phidot_setpoint,theta_setpoint;
float pitch_total=0,pitch_offset= - 1.84;
int pitch_value=1;
float pitch_control=0;
float R_phidot,phidot,phidot_f,phil;
float L_phidot;
float R_offset,L_offset;
 
void System(int , int, int);
void System_check(float , float, bool );


float phi_error;

int phi(){
  return (R_phi+L_phi)/2;
}


void setup()
{   
    Serial1.begin(9600);
    Serial.begin(38400);
    theta.limit(255);
    phi_set.limit(6.0);
    steer.limit(250);

    motor_setup();
    MPU_initialize();
    indicator_setup();

    
    
    noInterrupts();           // disable all interrupts
    timer1_init();
    timer5_init();
    interrupts();             // enable all interrupts
   
 }
  void phi_phidot_setpoint(int x, int y,float &phi_setpoint,float &phidot_setpoint, int &R_phi, int &L_phi, float &R_offset,float &L_offset){
   float Max;
  if(!mode){
    phidot_set.limit(6.0);
    phi_set.limit(4.0);
    digitalWrite(Blue_pin,LOW);
    Max=100;
  }
  else{
    phi_set.limit(2.0);
    Max = 20;
    if(abs(phidot_set.error)<10){
    phidot_set.limit(2.0);
    }    
    else{
    phidot_set.limit(4.0);
    }
    digitalWrite(Blue_pin,HIGH);
  }

    
    if(Centre(x,y))
    { phidot_set.limit(6.0);
      phidot_setpoint=0;  
       L_offset= 0;
     R_offset = 0;
    }
    else if(Forward(x,y))    
    { phidot_setpoint = Max;
      phi_setpoint += 5  ;
      L_offset= 0;
     R_offset = 0;
    }
    else if(Backward(x,y))
    { phi_setpoint -= 5 ;
      phidot_setpoint=-Max;
      L_offset= 0;
     R_offset = 0;
    }
    else if(Right(x,y)){
     R_phi=L_phi;
      L_offset=15;
     R_offset = -15;
    }
     else if(Left(x,y)){
      L_phi=R_phi;     
      R_offset= 15;
      L_offset = -15;
    }
}
 ISR(TIMER1_OVF_vect)        // interrupt service routine 
{ 

  TCNT1 = timer1_counter;   // preload timer
  complimentary(pitch,roll,ax,ay,az,gx,gy,gz);
  phidot = (phi() - phil)*6/(27*dT);
  phidot1.filter(phidot,0.9);
  phidot2.filter(phidot1.out,0.9);
  phil = phi();
  if((l%10)==0)
  phi_phidot_setpoint(x,y,phi_setpoint,phidot_setpoint,R_phi,L_phi,R_offset,L_offset);
  System(phi_setpoint,phidot_setpoint);
   l++;
}

 ISR(TIMER5_OVF_vect)        // interrupt service routine 
{
    if(sw == 1){
    start_indication();
    }
    
    else if(sw == 2){
    stop_indication();
    }

    
    else{
    digitalWrite(Green_pin,HIGH);
    digitalWrite(Red_pin,HIGH);
    digitalWrite(buzzer_pin,HIGH);
    }
    
    

    sw = 0;
}

void loop()
{   
    MPU_read();
    if(isnan(pitch))
    {
      MPU_initialize();
    }
    
    remote(x,y,MagF,MagB,present_val,mode);
    
    magnet(MagF,MagB);
  

if(previous_val==0){
  if(present_val==1){
    sw=1;
  }
}
else{
  if(present_val==0){
    sw=2;
  }
}

previous_val = present_val;


phi_set.change_constants(mode);
phidot_set.change_constants(mode);
steer.change_constants(mode);
theta.change_constants(mode);  
}


void System(int phi_setpoint,int phidot_setpoint){
  phi_set.PID_controller(phi_setpoint,phi(),dT);
  phidot_set.PID_controller(phidot_setpoint,phidot2.out,dT); 
  theta_setpoint=phidot_set.out + phi_set.out;   
  theta.PID_controller(theta_setpoint,(pitch-pitch_offset),dT);
  steer.PID_controller(-(R_phi),-(L_phi),dT);
  if((R_offset-L_offset)>0){
    steer.out=-20;
     L_phi=R_phi;
  }
  else if ((R_offset-L_offset)<0){
    steer.out=20;
     L_phi=R_phi;
  }
  System_check(theta.error,pitch,rotation_flag);
 
}



void System_off(){
  phi_set.reset();
  phidot_set.reset();
  theta.reset();
  steer.reset();
  motor_off();
  phi_setpoint=0;
 
}

void rotation_reset(){
  R_phi=0;
  L_phi=0;
  phi_setpoint=0;
  phi_set.reset();
  phidot_set.reset();
}

void System_check(float error, float pitch, bool flag){
  if((abs(pitch)>30)){
    System_off();
  }
  else {
    balance(theta.out,steer.out,R_offset,L_offset);
  }

}
