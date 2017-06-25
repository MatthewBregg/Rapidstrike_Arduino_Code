#include <Button_Debounce.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* OLED Display preprocessor */

// If using software SPI (the default case):
#define OLED_MOSI   10 // SDA GREEN
#define OLED_CLK   9 // SCK PINK
#define OLED_DC    12 // DC yellow
#define OLED_CS    13 // CS orange
#define OLED_RESET 11 // RST brown
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
/* End Oled Display PreProcessor stuff */

/* Pins for the controlling motors, and reading switches. */
const int motor_pin = A3;
const int nchan_flywheel_mosfet_pin = 5;

//These switches are active_low, aka, switch closed == low.
const int trigger_switch = 6;
const int cycle_switch = A4;
const int rev_switch = 8;
const int mag_switch = 4;
const int selector_switch_a = 0;
const int selector_switch_b = 0;
const int flashlight_pin = 0; 
const int touch_sensor_pin = 3; // NOTE : THIS IS TOUCHED WHEN HIGH (active_high)
const int fx_pin = A5;
const int voltage_pin = A2;
/* End the section on pins */

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
    //Pusher is stalled?! Turn off motor, hopefully before it gets damaged.
    pusher_was_stalled = true;
    set_motor(false);
    shots_to_fire = 0;
  }
}
void set_motor(bool on) {
  if ( pusher_was_stalled ) {
    // If we were stalled, turn off everything.
    digitalWrite(motor_pin,LOW); //Turn off motor

    motor_enabled = false;
    return; 
  } // If we were stalled, ensure the motor stays off until it gets cleared by a select switch, so make it like user turned off motor again. 
  if ( motor_enabled == on ) { return; } //Don't turn on / off if already on / off.
  if (on) {
    motor_enabled_at = millis(); // This is for the safety.
    digitalWrite(motor_pin,HIGH); //Turn on motor
    motor_enabled = true;
  } else { 
    digitalWrite(motor_pin,LOW); //Turn off motor
    motor_enabled = false;
  }
}


BasicDebounce trigger = BasicDebounce(trigger_switch, 20);
// Idea, perhaps have cycler, and ammo cycler objects.
// cycler would have a very low debounce delay, 
// and be used for fire control where bouncing might have to happen for good cycle control.
// Ammo cycler can have a high debounce, and be used for ammo counting.
BasicDebounce cycler = BasicDebounce(cycle_switch,13, HIGH);
BasicDebounce magazine_in = BasicDebounce(mag_switch,50);
BasicDebounce selector_a = BasicDebounce(selector_switch_a,50);
BasicDebounce selector_b = BasicDebounce(selector_switch_b,50);
BasicDebounce touch_sensor = BasicDebounce(touch_sensor_pin,30, HIGH);

void update_buttons() {
  trigger.update();
  cycler.update();
  magazine_in.update();
  selector_a.update();
  selector_b.update();
  touch_sensor.update();
}

void render_display(bool force_render);

uint8_t shots_fired = 0;
int burst_mode = 3; //How many shots to fire with each trigger pull.
enum FireMode  { burst_fire = 0, full_auto = 1 };
FireMode fire_mode = burst_fire;

void semi_auto_trigger_press_handler(BasicDebounce* button) {
  // Interruptable bursts, pulling the trigger again will restart the burst!
  // Also do N on the queue logic, essentially, allowing stacking up to N bursts from pulling the trigger. Going to start with one. 
  const int max_stack = (4-burst_mode)*burst_mode;
  if ( shots_to_fire < max_stack) { // Update this line if adding bigger bursts.
     shots_to_fire = min(shots_to_fire+burst_mode,max_stack);
     set_motor(true);
  }
}

void full_auto_trig_press_handler(BasicDebounce* button) {
  set_motor(true);
}


void full_auto_trig_release_handler(BasicDebounce* button) {
  // Change the following line to switch between live/dead center.  Commented out is live, in for dead.
  // Perhaps a good compromise would be, if trigger is released, keep motor on until cycle control is hit, but then don't turn
  // if back on if it overruns. 
  // set_motor(false); 
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

bool stealth_status = false;
bool handle_stealth_mode(BasicDebounce* button) {
  const uint16_t hold_time = 3000; //How many MS a button must be held down to trigger the flashlight
  // Determine if stealth mode should changee
  if (button->time_in_state() > hold_time ) {
    stealth_status = !stealth_status;
    return true;
  }
  return false;
}

bool handle_flashlight(BasicDebounce* button) { 
  static uint8_t flashlight_status = LOW; // Can just fully encapsulate this in function.
  const uint16_t flashlight_hold_time = 750; //How many MS a button must be held down to trigger the flashlight
  // Determine if flashlight should change
  if (button->time_in_state() > flashlight_hold_time ) {
    if ( flashlight_status == LOW ) { flashlight_status = HIGH; }
    else { flashlight_status = LOW; }
    digitalWrite(flashlight_pin,flashlight_status); // Set flashlight to be on/off
    return true;
  }
  return false;
}

void selector_b_handler(BasicDebounce* button) {
  clear_stall_safety();
  if ( fire_mode == burst_fire && burst_mode == 3) {
    set_full_auto();
  } else if (fire_mode == full_auto) {
    set_burst_fire(1);
  } else { ++burst_mode; }
  render_display(true);
}

void selector_a_handler(BasicDebounce* button) {
  clear_stall_safety();
  if ( fire_mode == burst_fire && burst_mode == 1 ) {
    set_full_auto();
  } else if (fire_mode == full_auto) {
    set_burst_fire(3);
  } else { --burst_mode; }
  render_display(true);
}

void selector_a_release_handler(BasicDebounce* button) {
  if ( handle_stealth_mode(button) ) { return; }
  if ( handle_flashlight(button) ) { return; }
  selector_a_handler(button);
}

void selector_b_release_handler(BasicDebounce* button) {
  if ( handle_stealth_mode(button) ) { return; }
  if ( handle_flashlight(button) ) { return; }
  selector_b_handler(button);
}

void setup_fs_buttons() {

  selector_a.set_released_command(&selector_a_release_handler);
  selector_b.set_released_command(&selector_b_release_handler);
}



void handle_pusher_retract(BasicDebounce* button) {
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
  if ( fire_mode == full_auto && !trigger.query() ) {
    set_motor(false); // Useful for retract on mag release function.
  }
  cycle_last_depressed_at = millis(); // Pusher got depressed, for safety
  render_display(true); // Pusher just left, safe to do a render
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
  //Display setup end
  // ----------------------------------------



  // Motor controls
  //---------------------------------------
  pinMode(motor_pin, OUTPUT); //Motor
  digitalWrite(motor_pin, LOW); 

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
  pinMode(touch_sensor_pin,INPUT_PULLUP);
  //---------------------------------------
  // Flash light
  pinMode(flashlight_pin,OUTPUT);

  // Volt meter
  //---------------------
   pinMode(voltage_pin, INPUT);
  //---------------------

  // FX
  //---------------------
  pinMode(fx_pin,OUTPUT);
  //-------------------


  // Set up fire_select buttons
  setup_fs_buttons();

  // Set up the cycle handler
  cycler.set_released_command(&handle_pusher_retract);

  // Set up trigger buttons & firing mode, default to 2 shot burst.
  set_burst_fire(2);
  
}

;
float calculate_voltage() {
//http://www.electroschematics.com/9351/arduino-digital-voltmeter/
  const float R1 = 68300.0; // resistance of R1 (75K) -see text!
  const float R2 = 9920.0; // resistance of R2 (10K) - see text!
  float value = analogRead(voltage_pin);
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

void render_ammo_counter() {
    if ( !magazine_in.query()) {
      // Can either fill the circle, draw an X, or display something like C.O. in circle. (Or graphic for mag out, but that's above me. 
      // I like the X idea best, simple, and easy to understand meaning.
      display.drawLine(display.width()/2-19,display.height()/2-19,display.width()/2+19, display.height()/2+19, WHITE);
      display.drawLine(display.width()/2+19,display.height()/2-19,display.width()/2-19, display.height()/2+19, WHITE);
      shots_fired = 0;
    } else {
      // Font width is 5, height is 8, * scale factor.
      const uint8_t scale_factor = 4;
      const uint8_t font_width = 5*scale_factor;
      const uint8_t font_height = 8*scale_factor;
      const char tens_place = '0'+shots_fired/10;
      const char ones_place = '0'+shots_fired%10;
      display.drawChar(display.width()/2-(font_width)-1, display.height()/2-(font_height/2)+2,tens_place,1,0,scale_factor); // Take the center of the screen, and shift over enough for 2 chars.
      display.drawChar(display.width()/2+3, display.height()/2-(font_height/2)+2,ones_place,1,0,scale_factor); // Take the center of the screen, and shift over enough for 1 chars.
    }
     display.drawCircle(display.width()/2,display.height()/2,31, WHITE);
}


void render_battery_indicator() {
  const float battery_min_voltage = 9;
  const float battery_full_voltage = 12.6;
  static int battery_percentage = 0;
  static byte voltage_to_print = 0; // Doing this to avoid float operations every single display update when printing, probably not needed though.
  // First handle updating voltage reading if enough time has passed!
    if ( millis() - last_updated_voltage_at > 512 || last_updated_voltage_at == 0 ) {
      voltage_to_disp = calculate_voltage();
      voltage_to_print = voltage_to_disp*10;
      last_updated_voltage_at = millis();
      // Max voltage will be considered to be 8.4, and min will be 7.0 (limit of regulator, and close limit of safe lipo usage.) If using this code with a different battery, 
      // change these constants if you want an accurate battery meter!!
      // Could probably avoid float altogther if really wanted.
      battery_percentage = 20*((voltage_to_disp-battery_min_voltage)/(battery_full_voltage-battery_min_voltage));
      if ( battery_percentage > 20 ) { battery_percentage = 20; }
      if ( battery_percentage < 0 ) { battery_percentage = 0; }
    }

  // hard code in a battery icon.
  display.drawRect(4,3,3,2,1);
  display.drawRect(0,5,11,20,1);
  display.fillRect(0,5,11,20-battery_percentage,1);
  
  // Now add a voltage display.
  display.setCursor(13, 2);
  display.setTextColor(1);
  display.setTextSize(1);
  display.print(voltage_to_print/10);
  display.print('.');
  display.print(voltage_to_print%10);
  display.print('V');
  if ( pusher_was_stalled ) {
    display.print('S');
  }
}

void draw_dart(byte x, byte y) {
  display.drawFastHLine(x,y,5,1); // Dart body
  display.drawFastHLine(x-3,y,2,1); // Dart tail

  
}

void render_firing_mode() {
  if ( fire_mode == burst_fire ) {
     for ( byte i = 0; i <= burst_mode; ++i ) {
      draw_dart(2*i+3,display.height()-3*i);
     }
     display.drawChar(15,display.height()-3*3, '0'+burst_mode, 1, 0, 1);
  } else { 
    display.setCursor(2, display.height()-3*3);
    display.setTextColor(1);
    display.setTextSize(1);
    display.print("AUTO");
  }
}
void render_display(bool force_render = false) {
  if ( stealth_status ) {
    if ( !motor_enabled ) {
      display.clearDisplay();
      display.display();
    }
    return;
  }
  if ( !motor_enabled || force_render) {
    display.clearDisplay();
    render_ammo_counter();
    render_battery_indicator();
    render_firing_mode();
    display.display();
  }
}

void retract_pusher_if_mag_out() {
  if (!magazine_in.query() && !cycler.query() && !pusher_was_stalled) {
    set_motor(true); // If mag is out, and pusher is extended, retract the motor. 
  }
}
void handle_fx() {
  if ( touch_sensor.query() ) {
    digitalWrite(fx_pin,HIGH);
  } else {
    digitalWrite(fx_pin,LOW);
  }
}
void loop() {
  pusher_safety_shutoff();
  handle_flywheels();
  update_buttons();
  render_display();
  handle_fx();
}

