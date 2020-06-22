int timer1_counter,timer5_counter;



void timer1_init()
{
  TCCR1A = 0;
  TCCR1B = 0;
  
  timer1_counter = 65224;   // preload timer 65536-16MHz/256/2Hz
  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void timer5_init()
{
  TCCR5A = 0;
  TCCR5B = 0;

  timer5_counter = 3036;   // preload timer 65536-16MHz/256/1Hz
  
  TCNT5 = timer5_counter;   // preload timer
  TCCR5B |= (1 << CS12);    // 256 prescaler 
  TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt 
}
