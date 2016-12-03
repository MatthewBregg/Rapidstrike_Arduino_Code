#include <LedControl.h>

/* Create a new LedControl variable.
 * We use pins 12,11 and 10 on the Arduino for the SPI interface
 * Pin 12 is connected to the DATA IN-pin of the first MAX7221
 * Pin 11 is connected to the CLK-pin of the first MAX7221
 * Pin 10 is connected to the LOAD(/CS)-pin of the first MAX7221
 * There will only be a single MAX7221 attached to the arduino 
 */  
LedControl lc=LedControl(9,10,11,1); 
const int led_mode_disp = 0;
const int led_counter_ones = 1;
const int led_counter_tens = 2;

const int mag_switch = 13;

// the setup function runs once when you press reset or power the board
bool prev_tick_pusher_switch_status = false; //During the last iteration of loop, what was the status of the pusher rod switch. (The one that tells me if the arm is extended/retracted.)
void setup() {
  //Leds 
  //wake up the MAX72XX from power-saving mode
  lc.shutdown(0,false); 
  //set a medium brightness for the Leds
  lc.setIntensity(0,8); 
  //Motor controls
  pinMode(6, OUTPUT); //Motor
  pinMode(4, OUTPUT);  //Brake
  pinMode(5, OUTPUT); 
  digitalWrite(5,LOW);
  digitalWrite(6, LOW); 
  digitalWrite(4, LOW); 

  //Inputs
  pinMode(0,INPUT); // fire select button
  pinMode(1,INPUT); //Trigger switch
  pinMode(8,INPUT); //Pusher switch
  pinMode(mag_switch,INPUT); //Magazine in switch, low when in.

  //Set the prev tick counter
  prev_tick_pusher_switch_status = digitalRead(8);

  //Volt meter
   pinMode(A3, INPUT);
}

// the loop function runs over and over again forever
// Motor is on when pin 6 is HIGH!!! BUT IT WORKKSSSS
//Brake is on when pin 4 is high!!
//DO NOT ALLOW BOTH TO BE ON AT ONCE
//Both can be low though, so whenever switching, always set whichever one is being switched to low to low first!!!


/** 
 *  Turns motor on or off, enabling the break whenever the motor is off.
 *  Always use this to control the motor, as it ensures there is a delay during the switch, and only allows one to be on at a time
 *  This avoids shorts.
 */
bool motor_enabled = false;
void set_motor(bool on) {
  if ( motor_enabled == on ) { return; } //Don't turn on / off if already on / off.
  if (on) {
    digitalWrite(4,LOW); //Disengage brake
    delayMicroseconds(100); //Wait long enough for mosfet to switch
    digitalWrite(6,HIGH); //Turn on motor
    motor_enabled = true;
  } else { 
    digitalWrite(6,LOW); //Turn off motor
    delayMicroseconds(100); //Wait long enough for mosfet to switch!
    digitalWrite(4,HIGH); //Turn on brakes.
    motor_enabled = false;
  }
}
int shots_to_fire = 0;
int burst_mode = 3; //How many shots to fire with each trigger pull.
bool full_auto = false;
unsigned long fs_last_change_time = 0;  // the last time the trigger was released
unsigned long fs_debounce_delay = 50;    // the debounce time; increase if the output flickers
bool fs_has_changed = false;
bool last_fs_state_debounce = false;
bool handle_fire_select_button() {
  int val = digitalRead(0);
  bool reading;
  if ( val == LOW ) {
      reading =  true;
  } else { 
      reading =  false;
  }
  
  if (reading != last_fs_state_debounce) {
    // reset the debouncing timer
    fs_last_change_time = millis();
  }

  if ((millis() - fs_last_change_time) > fs_debounce_delay) {

    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

       if ( reading && !fs_has_changed) {
        //take action here
        if ( full_auto ) {
          burst_mode = 1;
          full_auto = false;
        } else if ( burst_mode == 1 || burst_mode == 2 ) {
          ++burst_mode;
        } else if ( burst_mode == 3 ) {
          burst_mode = 1; //This is the then full auto and then some. Whatever value this is set to,
          // is the minimium darts fired per trig pull in FA. 
          full_auto = true;
        }
        fs_has_changed = true;
       } else if (!reading) {
          fs_has_changed = false;
       }
  }
  last_fs_state_debounce = reading;
  return reading;

}



unsigned long trigger_last_change_time = 0;  // the last time the trigger was released
unsigned long trigger_debounce_delay = 16;    // the debounce time; increase if the output flickers
bool has_fired = false;
bool last_trigger_state_debounce = false;
// True when trigger is pulled
bool handle_trigger() {
  int val = digitalRead(1);
  bool reading;
  if ( val == LOW ) {
      reading =  true;
  } else { 
      reading =  false;
  }
  
  if (reading != last_trigger_state_debounce) {
    // reset the debouncing timer
    trigger_last_change_time = millis();
  }

  if ((millis() - trigger_last_change_time) > trigger_debounce_delay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

       if ( reading && shots_to_fire == 0 && !has_fired) {
         // If we are not currently bursting, and if trigger goes from open to close, then fire shots
         shots_to_fire = burst_mode;
         has_fired = true;
       } else if ( shots_to_fire == 0 && !reading) {
          has_fired = false;
       }
  }
  last_trigger_state_debounce = reading;
    
  return reading;
}
unsigned long pusher_last_change_time = 0;  // the last time the pin was toggled
unsigned long pusher_debounce_delay = 50;    // the debounce time; increase if the output flickers

//True when pusher is all the way retracted, else false.
bool pusher_switch_status() {
  int val = digitalRead(8);
  bool current_val;
  if ( val == HIGH ) {
    current_val = true;
  } else {
    current_val = false;
  }
  if ( (millis() - pusher_last_change_time) > pusher_debounce_delay && current_val != prev_tick_pusher_switch_status) {
    pusher_last_change_time = millis();
    return current_val;
  } else { 
      return prev_tick_pusher_switch_status;
  }

}

int shots_fired = 0;
long last_updated_counter_meter_disp = 0;
long update_counter_meter_interval = 200;
long led_reset_counter = 0;
void loop() {
  if ( full_auto ) {
    // Set the fire_mode display
    lc.setDigit(0,led_mode_disp,(byte)10,false);
  } else {
     // Set the fire_mode display
     lc.setDigit(0,led_mode_disp,(byte)burst_mode,false);
  }
  
  bool fs_status = handle_fire_select_button();
  bool trigger_pulled = handle_trigger();
  // Handle updating the pusher status, and shots remaining to be fired
  // Also handle turning the pusher motor on / off.
  // My logic is that whenever pusher goes from open -> closed, that means it did a rotation and fired a shot.
  // Also don't let shots_to_fire go negative.
  bool curr_tick_pusher_switch_status = pusher_switch_status();
  if ( curr_tick_pusher_switch_status  && !prev_tick_pusher_switch_status ) {
      //We went from rod extended to rod closed
      shots_to_fire = shots_to_fire - 1;
      ++shots_fired;
      if ( shots_to_fire < 0 ) { shots_to_fire = 0; }
  }

  // IF the pusher switch is currently open, fire. If we still want to fire shots, then fire.
  // If full auto and trigger pulled, the nfire 
  // Otherwise don't fire.
  if ( (!curr_tick_pusher_switch_status || shots_to_fire > 0) || ( full_auto && trigger_pulled ) ){
    set_motor(true);
  } else {
    set_motor(false);
  }
  if ( ( millis() - last_updated_counter_meter_disp ) > update_counter_meter_interval ) {
    ++led_reset_counter;
    if ( led_reset_counter%5 == 0 ) {
     // digitalWrite(5,LOW);
     // digitalWrite(5,HIGH);
      lc=LedControl(9,10,11,1); 
      lc.shutdown(0,false); 
      //set a medium brightness for the Leds
      lc.setIntensity(0,8);
    }
      
     last_updated_counter_meter_disp = millis();
     int magazine_in = digitalRead(mag_switch);
    //Handle the counter
    if ( !fs_status ) {
      if ( magazine_in == LOW ) {
        lc.setDigit(0,led_counter_tens,(byte)shots_fired/10,false);
        lc.setDigit(0,led_counter_ones,(byte)shots_fired%10,false);
      } else {
        //If clip is out, aka, magazine switch is high, then reset fired count to 0
        lc.setDigit(0,led_counter_tens,(byte)12,true);
        lc.setDigit(0,led_counter_ones,(byte)0,true);
        shots_fired = 0;
      }
    } else {
      //http://www.electroschematics.com/9351/arduino-digital-voltmeter/
      const float R1 = 100000.0; // resistance of R1 (100K) -see text!
      const float R2 = 10000.0; // resistance of R2 (10K) - see text!
      float value = analogRead(A3);
      float vout = (value * 5.0) / 1024.0; // see text
      float vin = vout / (R2/(R1+R2)); 
      if (vin<0.09) {
        vin=0.0;//statement to quash undesired reading !
      }
      lc.setDigit(0,led_counter_tens,(byte)vin%10,true);
      lc.setDigit(0,led_counter_ones,(byte)(vin*10)%10,false);
    }
    lc.shutdown(0,true);
    lc.shutdown(0,false); //If display got corrupted by noise, see if this fixes it. 
  }
  
  //Update previous switch state vars
   prev_tick_pusher_switch_status = curr_tick_pusher_switch_status;

}
