#include <Button_Debounce.h>

 // Refactoring stuff
 // - move constants and defines into it's own headers.
 // - make display better

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

/** 
 *  Motor is on when pin pchan_motor_mosfet_pin is HIGH!!!
 *  Brake is on when pin nchan_motor_brake_mosfet_pin is high!!
 *  DO NOT ALLOW BOTH TO BE ON AT ONCE
 *  Both can be low though, so whenever switching, 
 *  always set whichever one is being switched to low to low first!!!
 */ 

int shots_to_fire = 0;
/** 
 *  Turns motor on or off, enabling the break whenever the motor is off.
 *  Always use this to control the motor, as it ensures there is a delay during the switch, and only allows one to be on at a time
 *  This avoids shorts.
 */
bool motor_enabled = false;
long motor_enabled_at = 0;
long cycle_last_depressed_at = 0;
bool pusher_was_stalled = false;
void clear_stall_safety() {
  pusher_was_stalled = false;
}
const int safety_interval = 250; //How many milliseconds to let the motor run without the cycle switched being hit before shutting it off.
void pusher_safety_shutoff() {
  bool stalled = ((millis() - cycle_last_depressed_at) > safety_interval) && ((millis() - motor_enabled_at)  > safety_interval);
  // Logic is : If it has been n time since the motor was enabled, and in that time, the pusher has not cycled, we have a stall.
  // can't just do millis() - cycle_last_depressed_at > safety_interval, because if pusher is resting halfway out, then that will be negative. 
  if ( motor_enabled && stalled ) {
    set_motor(false); //Pusher is stalled?! Turn off motor, hopefully before it gets damaged.
    pusher_was_stalled = true;
    shots_to_fire = 0;
  }
}
void set_motor(bool on) {
  if ( pusher_was_stalled ) { on = false; } // If we were stalled, ensure the motor stays off until it gets cleared by a select switch, so make it like user turned off motor again. 
  if ( motor_enabled == on ) { return; } //Don't turn on / off if already on / off.
  if (on) {
    motor_enabled_at = millis(); // This is for the safety.
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


BasicDebounce trigger = BasicDebounce(trigger_switch, 20);
BasicDebounce cycler = BasicDebounce(cycle_switch,4);
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

int shots_fired = 0;
int burst_mode = 3; //How many shots to fire with each trigger pull.
enum FireMode  { burst_fire = 0, full_auto = 1 };
FireMode fire_mode = burst_fire;

void semi_auto_trigger_press_handler(BasicDebounce* button) {
  if ( shots_to_fire == 0 ) {
    shots_to_fire += burst_mode;
    set_motor(true);
  }
}

void full_auto_trig_press_handler(BasicDebounce* button) {
  set_motor(true);
}

void full_auto_trig_release_handler(BasicDebounce* button) {
  set_motor(false);
}

// If true, then fire full_auto, else burst fire
void set_full_auto() {
    burst_mode = 0;
    fire_mode = full_auto;
    shots_to_fire = 0;
    set_motor(false);
    trigger.set_pressed_command(&full_auto_trig_press_handler);
    trigger.set_released_command(&full_auto_trig_release_handler); 
}

void set_burst_fire(byte mode) {
  burst_mode = mode;
  fire_mode = burst_fire;
  shots_to_fire = 0;
  set_motor(false);
  trigger.set_pressed_command(&semi_auto_trigger_press_handler); 
  trigger.set_released_command(0);
}



void selector_b_handler(BasicDebounce* button) {
  clear_stall_safety();
  if ( fire_mode == burst_fire && burst_mode == 3) {
    set_full_auto();
  } else if (fire_mode == full_auto) {
    set_burst_fire(1);
  } else { ++burst_mode; }
}

void selector_a_handler(BasicDebounce* button) {
  clear_stall_safety();
  if ( fire_mode == burst_fire && burst_mode == 1 ) {
    set_full_auto();
  } else if (fire_mode == full_auto) {
    set_burst_fire(3);
  } else { --burst_mode; }
}

void setup_fs_buttons() {
  selector_a.set_pressed_command(&selector_a_handler);
  selector_b.set_pressed_command(&selector_b_handler);
}


void handle_pusher_retract(BasicDebounce* button) {
  cycle_last_depressed_at = millis(); // Pusher got depressed, for safety
  // Keep track of shots to fire and ammo count
  ++shots_fired;
  --shots_to_fire;
  if ( shots_to_fire <  0 ) {
    shots_to_fire = 0;
  }
  // Disable pusher if done burst firing
  if ( shots_to_fire  == 0 && fire_mode == burst_fire ) {
    set_motor(false);
  }
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


  // Set up fire_select buttons
  setup_fs_buttons();

  // Set up the cycle handler
  cycler.set_pressed_command(&handle_pusher_retract);

  // Set up trigger buttons & firing mode, default to 2 shot burst.
  set_burst_fire(2);
  
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

void handle_flywheels() {
   digitalWrite(nchan_flywheel_mosfet_pin,!digitalRead(rev_switch));
}

float voltage_to_disp = 0.0;
unsigned long last_updated_voltage_at = 0;



void loop() {
  pusher_safety_shutoff();
  handle_flywheels();
  update_buttons();
  digitalWrite(flashlight_pin,digitalRead(selector_switch_b));


  if ( !motor_enabled ) {
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
  if ( fire_mode == full_auto ) {
   display.println("full auto");
  } else {
    display.print(burst_mode);
    display.print("-burst");
  }
  display.display();
 }
  
  
}

