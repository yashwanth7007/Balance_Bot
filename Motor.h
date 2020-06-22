#define InR1            32                    // motor pin
#define PWMR            6                      // PWM motor pin                                                                                                                 
#define InR2            34                       // motor pin  


#define InL1            28                       // motor pin
#define PWML            5                       // PWM motor pin
#define InL2            30                       // motor pin

#define dead 60



#define right_phi_A_interrupt  2  
#define right_B               24

#define left_phi_A_interrupt 3  
#define left_B               38

int R_phi;
int L_phi;

void right_balance(int);
void left_balance(int);

void right_phi(){
  if(digitalRead(right_B)){
    R_phi--;
  }
  else{
    R_phi++;
  }
}

void left_phi(){
  if(digitalRead(left_B)){
    L_phi++;
  }
  else{
    L_phi--;
  }
}

void motor_setup()
{
   pinMode(InL1, OUTPUT);
   pinMode(InL2, OUTPUT);
   pinMode(PWML, OUTPUT);
    
   pinMode(InR1, OUTPUT);
   pinMode(InR2, OUTPUT);
   pinMode(PWMR, OUTPUT);

   pinMode(right_phi_A_interrupt, INPUT_PULLUP);
   pinMode(left_phi_A_interrupt, INPUT_PULLUP);
   
   attachInterrupt(digitalPinToInterrupt(right_phi_A_interrupt),right_phi ,FALLING);
   attachInterrupt(digitalPinToInterrupt(left_phi_A_interrupt),left_phi ,FALLING);
    
    pinMode(right_B,INPUT);
    pinMode(left_B,INPUT);
}




void Right_forward(int pwm){
 analogWrite(PWMR,pwm);
 digitalWrite(InR2,LOW);
 digitalWrite(InR1,HIGH); 
}

void Right_backward(int pwm){
 analogWrite(PWMR,pwm);
 digitalWrite(InR1,LOW);
 digitalWrite(InR2,HIGH);
 }

void Right_off(){
  analogWrite(PWMR,0);
  digitalWrite(InR1,LOW);
  digitalWrite(InR2,LOW);  
 }

void Left_off(){
  analogWrite(PWML,0);
  digitalWrite(InL1,LOW);
  digitalWrite(InL2,LOW);  
 }

void Left_forward(int pwm){
 analogWrite(PWML,pwm);
 digitalWrite(InL2,LOW);
 digitalWrite(InL1,HIGH);
}

void Left_backward(int pwm){
 analogWrite(PWML,pwm);
 digitalWrite(InL1,LOW);
 digitalWrite(InL2,HIGH);
 }

 void balance(int control, float phi_error, float R, float L){
  
  right_balance(control-phi_error);
  left_balance(control+phi_error);
}


void right_balance(int control){
  if(control<0){
  int x = map(control,-255,0,-255,-dead);
  
  Right_backward(-x);
  }
 else
  {
    int x = map(control,0,255,dead,255);
    Right_forward(x);
   
  }

  
}

void left_balance(int control){
if(control<0){
  int x = map(control,-255,0,-255,-dead);
  Left_backward(-x);
  }
 else
  {
   int x = map(control,0,255,dead,255);
   Left_forward(x); 
    
}
}

void motor_off(){
  Right_off();
  Left_off();
}



  
