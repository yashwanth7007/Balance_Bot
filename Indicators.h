#define Mag_front 42
#define Mag_back 44
#define buzzer_pin 22
#define Red_pin 47
#define Green_pin 49 
#define Blue_pin 51
#define Common_pin 53



void indicator_setup()
{
  pinMode(buzzer_pin,OUTPUT);
  pinMode(Common_pin,OUTPUT);
  pinMode(Red_pin,OUTPUT);
  pinMode(Green_pin,OUTPUT);
  pinMode(Blue_pin,OUTPUT);

  pinMode(Mag_front,OUTPUT);
  pinMode(Mag_back,OUTPUT);
  
  digitalWrite(Mag_front,LOW);  
  digitalWrite(Mag_back,LOW);
  
  digitalWrite(buzzer_pin,HIGH);
  digitalWrite(Red_pin,HIGH);
  digitalWrite(Green_pin,HIGH);
  digitalWrite(Blue_pin,HIGH);
  digitalWrite(Common_pin,HIGH);
  
}

void start_indication(void)
{
  digitalWrite(Green_pin,LOW);
  digitalWrite(buzzer_pin,LOW);
  digitalWrite(Red_pin,HIGH);
}

void stop_indication(void)
{ digitalWrite(Green_pin,HIGH);
  digitalWrite(Red_pin,LOW);
  digitalWrite(buzzer_pin,LOW);
}


void magnet(int magF,int magB)
{
   if(magF)
   {  
    digitalWrite(Mag_front,HIGH);
   }
   else if(magF == 0)
   {
    digitalWrite(Mag_front,LOW);
   }

   if(magB)
   {
    digitalWrite(Mag_back,HIGH);
   }
   else if(magB == 0)
   {    
    digitalWrite(Mag_back,LOW);
   }
   
   
}
