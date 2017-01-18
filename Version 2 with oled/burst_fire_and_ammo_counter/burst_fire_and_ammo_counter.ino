#include <Button_Debounce.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);



#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 


#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
// End OLED init stuff

bool prev_tick_pusher_switch_status = false; 
// During the last iteration of loop,
// what was the status of the pusher rod switch. 
// (The one that tells me if the arm is extended/retracted.)
const int pchan_motor_mosfet_pin = 6;
const int nchan_motor_brake_mosfet_pin = 3;
const int nchan_flywheel_mosfet_pin = A5;

//These switches are active_low, aka, switch closed == low.
const int trigger_switch = 4;
const int cycle_switch = 5;
const int rev_switch = 8;
const int mag_switch = A4;
const int selector_switch_a = A0;
const int selector_switch_b = A1;
const int flashlight_pin = A2; 

BasicDebounce trigger = BasicDebounce(trigger_switch, 20);
BasicDebounce cycler = BasicDebounce(cycle_switch,30);
BasicDebounce magazine_in = BasicDebounce(mag_switch,50);
BasicDebounce selector_a = BasicDebounce(selector_switch_a,50);
BasicDebounce selector_b = BasicDebounce(selector_switch_b,50);

void update_buttons() {
  trigger.update();
  cycler.update();
  magazine_in.update();
  selector_a.update();
  selector_b.update();
}

int shots_to_fire = 0;
int burst_mode = 3; //How many shots to fire with each trigger pull.
bool full_auto = false;
void selector_b_handler(BasicDebounce* button) {
  if ( burst_mode == 3 ) {
    full_auto = true;
    burst_mode = 0;
  } else if (full_auto) {
    burst_mode = 1;
    full_auto = false;
  } else { ++burst_mode; }
}

void selector_a_handler(BasicDebounce* button) {
    if ( burst_mode == 1 ) {
    full_auto = true;
    burst_mode = 0;
  } else if (full_auto) {
    burst_mode = 3;
    full_auto = false;
  } else { --burst_mode; }
}

void setup_fs_buttons() {
  selector_a.set_pressed_command(&selector_a_handler);
  selector_b.set_pressed_command(&selector_b_handler);
}

void setup()   {   
  //Display Set up begins 
  //--------------------------------------------------             
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(100); 
  //display.setRotation(2); //rotate display
  //Display setup end
  // ----------------------------------------



  // Motor controls
  //---------------------------------------
  pinMode(pchan_motor_mosfet_pin, OUTPUT); //Motor
  pinMode(nchan_motor_brake_mosfet_pin, OUTPUT);  //Brake
  digitalWrite(nchan_motor_brake_mosfet_pin, LOW); 
  digitalWrite(pchan_motor_mosfet_pin, LOW); 

  pinMode(nchan_flywheel_mosfet_pin, OUTPUT);
  digitalWrite(nchan_flywheel_mosfet_pin,LOW);
  //--------------------------------------------

  // Switch inputs
  //---------------------------------------
  pinMode(trigger_switch, INPUT_PULLUP);
  pinMode(cycle_switch, INPUT_PULLUP);
  pinMode(rev_switch, INPUT_PULLUP);
  pinMode(mag_switch, INPUT_PULLUP);
  pinMode(selector_switch_a, INPUT_PULLUP);
  pinMode(selector_switch_b, INPUT_PULLUP);
  //---------------------------------------
  // Flash light
  pinMode(flashlight_pin,OUTPUT);

  // Volt meter
  //---------------------
   pinMode(A3, INPUT);
  //---------------------

  
  //Set the prev tick counter
  prev_tick_pusher_switch_status = digitalRead(cycle_switch);


  // Set up fire_select buttons
  setup_fs_buttons();
}




// Motor is on when pin pchan_motor_mosfet_pin is HIGH!!!
//Brake is on when pin nchan_motor_brake_mosfet_pin is high!!
//DO NOT ALLOW BOTH TO BE ON AT ONCE
//Both can be low though, so whenever switching, 
//always set whichever one is being switched to low to low first!!!


/** 
 *  Turns motor on or off, enabling the break whenever the motor is off.
 *  Always use this to control the motor, as it ensures there is a delay during the switch, and only allows one to be on at a time
 *  This avoids shorts.
 */
bool motor_enabled = false;
void set_motor(bool on) {
  if ( motor_enabled == on ) { return; } //Don't turn on / off if already on / off.
  if (on) {
    digitalWrite(nchan_motor_brake_mosfet_pin,LOW); //Disengage brake
    delayMicroseconds(100); //Wait long enough for mosfet to switch
    digitalWrite(pchan_motor_mosfet_pin,HIGH); //Turn on motor
    motor_enabled = true;
  } else { 
    digitalWrite(pchan_motor_mosfet_pin,LOW); //Turn off motor
    delayMicroseconds(100); //Wait long enough for mosfet to switch!
    digitalWrite(nchan_motor_brake_mosfet_pin,HIGH); //Turn on brakes.
    motor_enabled = false;
  }
}

float calculate_voltage() {
//http://www.electroschematics.com/9351/arduino-digital-voltmeter/
  const float R1 = 75000.0; // resistance of R1 (75K) -see text!
  const float R2 = 10000.0; // resistance of R2 (10K) - see text!
  float value = analogRead(A3);
  float vout = (value * 5.0) / 1024.0; // see text
  float vin = vout / (R2/(R1+R2)); 
  if (vin<0.09) {
    vin=0.0;//statement to quash undesired reading !
  }
  return vin;
}




unsigned long trigger_last_change_time = 0;  // the last time the trigger was released
unsigned long trigger_debounce_delay = 30;    // the debounce time; increase if the output flickers
bool has_fired = false;
bool last_trigger_state_debounce = false;
// True when trigger is pulled
bool handle_trigger() {
  int val = digitalRead(trigger_switch);
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
unsigned long pusher_debounce_delay = 4;    // the debounce time; increase if the output flickers

//True when pusher is all the way retracted, else false.
bool pusher_switch_status() {
  int val = digitalRead(cycle_switch);
  bool current_val;
  if ( val == HIGH ) {
    current_val = false;
  } else {
    current_val = true;
  }
  if ( (millis() - pusher_last_change_time) > pusher_debounce_delay && current_val != prev_tick_pusher_switch_status) {
    pusher_last_change_time = millis();
    return current_val;
  } else { 
      return prev_tick_pusher_switch_status;
  }

}

void handle_flywheels() {
   digitalWrite(nchan_flywheel_mosfet_pin,!digitalRead(rev_switch));
}

float voltage_to_disp = 0.0;
unsigned long last_updated_voltage_at = 0;
int shots_fired = 0;

void loop() {
  update_buttons();
    digitalWrite(flashlight_pin,digitalRead(selector_switch_b));
  if ( millis() < 1000 ) {
    shots_fired = 0;
  }
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

  //  If we still want to fire shots, then fire.
  // If full auto and trigger pulled, then fire
  // Hmm, either this switch, or this has weaker breaking, but I'm getting run aways. 
  // Moving to dead center. 
  // Otherwise don't fire.
  if ( (shots_to_fire > 0) || ( full_auto && trigger_pulled )){
    set_motor(true);
  } else {
    set_motor(false);
  }

  //Update previous switch state vars
   prev_tick_pusher_switch_status = curr_tick_pusher_switch_status;


//Display code
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if ( magazine_in.query() ) {
  display.print("shots fired ");
  display.println(shots_fired);
  } else {
   display.println("MAG OUT");
   shots_fired = 0;
  }


  
  //Print voltage also, only update voltage on an fixed interval to avoid flicker
  if ( millis() - last_updated_voltage_at > 512 || last_updated_voltage_at == 0 ) {
    voltage_to_disp = calculate_voltage();
    last_updated_voltage_at = millis();
    }
  display.print("Voltage is ");
  display.println(voltage_to_disp);
  display.print("Fire Mode- ");
  if ( full_auto ) {
   display.println("full auto");
  } else {
    display.print(burst_mode);
    display.print("-burst");
  }
  display.display();
  
  handle_flywheels();
  
}

