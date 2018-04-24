void setup(){
 
  //pin 2 trigger switch
  pinMode(3, INPUT_PULLUP);
 
  //pin 10 flywheel motor controller PWM throttle signal
  pinMode(10, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1B = 230; //write 920us low throttle
 
}

void loop(){

  if(digitalRead(3) == LOW){
    //start flywheels
    OCR1B = 500; //go
    digitalWrite(LED_BUILTIN,HIGH);
  } else {
    OCR1B = 230; //shutdown
    digitalWrite(LED_BUILTIN,LOW);
  }
  delay(5);
}

