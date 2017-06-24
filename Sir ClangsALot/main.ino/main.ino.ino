/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  
  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  modified 2 Sep 2016
  by Arturo Guadalupi
*/

// 11 is output to wheels
// 12 is output to pusher
// 10 is input from rev trig.
// 9 is input from accel trig
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);

  pinMode(8,INPUT_PULLUP); // Rocker switch
  pinMode(7,INPUT_PULLUP); // Rocker switch
}

enum FIRINGMODE {LOCKED, DIRECT_SEMI, AUTO};

FIRINGMODE firing_mode = LOCKED;

void set_firing_mode() {
  FIRINGMODE old_firing_mode = firing_mode;
  if ( digitalRead(8) == HIGH && digitalRead(7) == HIGH ) {
    firing_mode = DIRECT_SEMI;
    if (old_firing_mode != firing_mode) {
      set_rod(LOW);
    }
    return;
  }

  if ( digitalRead(8) ==  LOW ) {
    firing_mode = AUTO;
    if (old_firing_mode != firing_mode) {
      set_rod(LOW);
    }
    return;
  }

  if ( digitalRead(7) == LOW ) {
    firing_mode = LOCKED;
    if (old_firing_mode != firing_mode) {
      set_rod(LOW);
    }
    return;
  }

  firing_mode = LOCKED;
  if (old_firing_mode != firing_mode) {
      set_rod(LOW);
    }
  return;
}
int invert_reading(int reading) {
  if ( reading == HIGH ) {
    return LOW;
  }
  return HIGH;
}

void set_rod(int reading) {
  // high for extended, low for retracted
  digitalWrite(12,reading);
  digitalWrite(LED_BUILTIN,reading);
}
void push_once() {
  set_rod(HIGH);
  delay(60); //45 too low, 55 a bit too much, but gives lee way
  set_rod(LOW);
  delay(90); // I think I can go lower to like 75 at least, but not sure. Need to get wired up befpore testing. 
}
// the loop function runs over and over again forever
void loop() {
  if ( digitalRead(10) == LOW && firing_mode != LOCKED) { // Don't rev if locked.
    digitalWrite(11,HIGH);
  } else {
    digitalWrite(11,LOW);
  }
  set_firing_mode();
  
 // Do nothing
  if ( firing_mode == DIRECT_SEMI ) {
    set_rod(invert_reading(digitalRead(9)));
  } else if ( firing_mode == AUTO ) {
    if ( digitalRead(9) == LOW ) {
      push_once();
    }
  } else {
    //locked or something, do nothing
  }

}

