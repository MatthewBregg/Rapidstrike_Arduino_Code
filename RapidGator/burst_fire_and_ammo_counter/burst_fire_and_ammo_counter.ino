#include <Button_Debounce.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* OLED Display preprocessor */

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   A2
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
/* End Oled Display PreProcessor stuff */

/* Pins for the controlling motors, and reading switches. */
const int pchan_motor_mosfet_pin = 6;
const int nchan_motor_brake_mosfet_pin = 3;
const int nchan_flywheel_mosfet_pin = A5;

//These switches are active_low, aka, switch closed == low.
const int trigger_switch = 4;
const int voltimeter_pin = A3;
const int other_trigger_switch = 1;
const int cycle_switch = 5;
const int rev_switch = 8;
const int mag_switch = A4;
const int selector_switch_a = A0;
const int selector_switch_b = A1;
const int flashlight_pin = 10; 
/* End the section on pins */

/** 
 *  Motor is on when pin pchan_motor_mosfet_pin is HIGH!!!
 *  Brake is on when pin nchan_motor_brake_mosfet_pin is high!!
 *  DO NOT ALLOW BOTH TO BE ON AT ONCE
 *  Both can be low though, so whenever switching, 
 *  always set whichever one is being switched to low to low first!!!
 */ 


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
  }
}
void set_motor(bool on) {
  if ( pusher_was_stalled ) {
    // If we were stalled, turn off everything.
    digitalWrite(pchan_motor_mosfet_pin,LOW); //Turn off motor
    digitalWrite(nchan_motor_brake_mosfet_pin,LOW); //Turn off brakes
    motor_enabled = false;
    return; 
  } // If we were stalled, ensure the motor stays off until it gets cleared by a select switch, so make it like user turned off motor again. 
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


BasicDebounce trigger = BasicDebounce(trigger_switch, 8);
// Idea, perhaps have cycler, and ammo cycler objects.
// cycler would have a very low debounce delay, 
// and be used for fire control where bouncing might have to happen for good cycle control.
// Ammo cycler can have a high debounce, and be used for ammo counting.
BasicDebounce cycler = BasicDebounce(cycle_switch,15);
BasicDebounce magazine_in = BasicDebounce(mag_switch,50);
BasicDebounce selector_a = BasicDebounce(selector_switch_a,50);
BasicDebounce selector_b = BasicDebounce(selector_switch_b,50);
BasicDebounce rev_switch_button = BasicDebounce(rev_switch,30);

void update_buttons() {
  trigger.update();
  cycler.update();
  magazine_in.update();
  selector_a.update();
  selector_b.update();
  rev_switch_button.update();
}

void render_display(bool force_render);

uint8_t shots_fired = 0;

void test_feed_delay(int initial_rev, int cool_down, int reheat) {

  digitalWrite(nchan_flywheel_mosfet_pin,HIGH);
  delay(initial_rev);
  digitalWrite(nchan_flywheel_mosfet_pin,LOW);
  delay(cool_down);
  digitalWrite(nchan_flywheel_mosfet_pin,HIGH);
  delay(reheat);
  return;
}

bool is_revving = false;
long turned_off_flywheels_at = 0;
short feed_delay_modifier = 0;
void block_and_rev_flywheels() {
  // NOTE: This was done with a 42 mm cage, using blasterparts wheels + morpheus, for an endwar ideal set up.
  // - 125 FPS ave with accufakes
  // - ~110-114 with brick tips, and probably also with waffles?
  
  if ( is_revving ) {
    // We released and depressed the trigger mid cycle, 
    // we never stopped firing, this was alreay ran, take no action.
    return;
  }

  // with 200 feed delay, got ave 115, and several 118/117 readings. Also one 90 reading.

  /**
   * Use a snippet like this to enable easily and accurately testing
   * feed delay
   * test_feed_delay(0,0,0);
   * return;
  **/

    // Note!! I realized a major mistake I've been making! I haven't been including pusher time when testing feed delays!
  // This means that when we cool down and then reheat to see if the shot is still hot, we aren't fully revved!
  //  That means all the FDs can be much lower/longer.
  // I should go back and correct this mistake for the rapid gator!


  
  // Otherwise, calculate delay, rev, delay, and return;
  short feed_delay = 145; // -8 Still works pretty well, perhaps a tiny bit more cold shots. // -16 same, cold shots like 1/5?
  // Could prob reduce at least -8 on storage voltage +, but to be on safe side, just go to 145.
  // However, when below (7.2V vs 7.4V), expect suboptimal operation unless feed delay is manually incremented!s
  
  if ( ( millis() - turned_off_flywheels_at) < 900 ) {
    feed_delay = 95;
  }

  if ( ( millis() - turned_off_flywheels_at) < 800 ) {
    feed_delay = 85;
  }

  if ( ( millis() - turned_off_flywheels_at) < 700 ) {
    feed_delay = 80;
  }

  if ( ( millis() - turned_off_flywheels_at) < 500 ) {
    feed_delay = 70;
  }

  if ( ( millis() - turned_off_flywheels_at) < 400 ) {
    feed_delay = 60;
  }

  if ( ( millis() - turned_off_flywheels_at) < 300 ) {
    feed_delay = 40;
  }

  if ( ( millis() - turned_off_flywheels_at) < 200 ) {
    feed_delay = 30;
  }

  if ( ( millis() - turned_off_flywheels_at) < 100 ) {
    // 10 too low.
    feed_delay = 20;
  }
  if ( (millis()-turned_off_flywheels_at) < 50 ) {
    // Barely stopped revving, so barely do a delay
    feed_delay = 12.5;
  }
  digitalWrite(nchan_flywheel_mosfet_pin,HIGH);
  is_revving = true;
  delay(feed_delay+feed_delay_modifier);
}

void finish_revving_flywheels() {
  turned_off_flywheels_at = millis();
  digitalWrite(nchan_flywheel_mosfet_pin,LOW);
  is_revving = false;
}

void full_auto_trig_press_handler(BasicDebounce* button) {
  block_and_rev_flywheels();
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
    set_motor(false);
    trigger.set_pressed_command(&full_auto_trig_press_handler);
    trigger.set_released_command(&full_auto_trig_release_handler); 
}


bool stealth_status = false;
bool handle_stealth_mode(BasicDebounce* button) {
  const uint16_t hold_time = 2000; //How many MS a button must be held down to trigger the stealth mode.
  // Determine if stealth mode should changee
  if (button->time_in_state() > hold_time ) {
    stealth_status = !stealth_status;
    return true;
  }
  return false;
}

constexpr byte feed_delay_change_value = 8;
bool handle_feed_delay_increase_modifier(BasicDebounce* button) {
   const uint16_t hold_time = 1000; //How many MS a button must be held down to trigger the stealth mode.
  // Determine if stealth mode should change
  if (button->time_in_state() > hold_time ) {
    feed_delay_modifier += feed_delay_change_value;
    return true;
  }
  return false;
}


bool handle_feed_delay_decrease_modifier(BasicDebounce* button) {
   const uint16_t hold_time = 1000; //How many MS a button must be held down to trigger the stealth mode.
  // Determine if stealth mode should change
  if (button->time_in_state() > hold_time ) {
    feed_delay_modifier -= feed_delay_change_value;
    return true;
  }
  return false;
}

// This MUST be a power of 2 with the current increment and decrement methods!
uint16_t current_flashlight_brightness = 64; 
uint8_t flashlight_status = LOW; 
bool handle_flashlight(BasicDebounce* button) { 
    // Switch the status from low to high or vice versa
    if ( flashlight_status == LOW ) { flashlight_status = HIGH; }
    else { flashlight_status = LOW; }

    // Based on that status, enable/disable the flashlight at the current power.
    if (flashlight_status == HIGH) { // Set flashlight on/off.
      //Turn on flashlight, set to current brightness
      analogWrite(flashlight_pin,current_flashlight_brightness); 
    } else {
      // Turn off flashlight by setting the PWM cycle to 0% duty.
      analogWrite(flashlight_pin, 0);
    }

    return true;
}

void set_flashlight_to_brightness() {
  // If flashlight is on, redo analog write to set current PWM duty.
  if (flashlight_status == HIGH) {
    analogWrite(flashlight_pin,current_flashlight_brightness);
  }
}

void increment_flashlight_brightness() {
  if ( current_flashlight_brightness == 255 ) {
    return;
  }

  if (current_flashlight_brightness == 0) {
    current_flashlight_brightness = 1;
    return;
  }
  
  current_flashlight_brightness *= 2;
  if (current_flashlight_brightness == 256) {
    current_flashlight_brightness = 255;
  }
}

void decrement_flashlight_brightness() {
  if ( current_flashlight_brightness == 255 ) {
    ++current_flashlight_brightness;
  }
  // Enabling letting the flashlight brightness go to 0 as a way of "locking out"
  // the flashlight to prevent accidental use.
  current_flashlight_brightness/=2;
}

// Forwards switch
void selector_b_handler(BasicDebounce* button) {
  clear_stall_safety();
  increment_flashlight_brightness();
  set_flashlight_to_brightness();
  render_display(true);
}

// Backwards switch
void selector_a_handler(BasicDebounce* button) {
  clear_stall_safety();
  decrement_flashlight_brightness();
  set_flashlight_to_brightness();
  render_display(true);
}

void selector_a_release_handler(BasicDebounce* button) {
  if ( handle_stealth_mode(button) ) { return; }
  if ( handle_feed_delay_decrease_modifier(button) ) { return; }
  selector_a_handler(button);
}

void selector_b_release_handler(BasicDebounce* button) {
  if ( handle_stealth_mode(button) ) { return; }
  if ( handle_feed_delay_increase_modifier(button) ) { return; }
  selector_b_handler(button);
}

void setup_fs_buttons() {

  selector_a.set_released_command(&selector_a_release_handler);
  selector_b.set_released_command(&selector_b_release_handler);
}



void handle_pusher_retract(BasicDebounce* button) {
  // Keep track of shots to fire and ammo count
  ++shots_fired;
  if (!trigger.query() ) {
    set_motor(false); // Useful for retract on mag release function.
    finish_revving_flywheels();
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
  display.setRotation(0); //rotate display
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
  pinMode(other_trigger_switch, INPUT_PULLUP);
  pinMode(cycle_switch, INPUT_PULLUP);
  pinMode(rev_switch, INPUT_PULLUP);
  pinMode(mag_switch, INPUT_PULLUP);
  pinMode(selector_switch_a, INPUT_PULLUP);
  pinMode(selector_switch_b, INPUT_PULLUP);
  //---------------------------------------

  
  // Set up trigger as dual pin
  trigger.AddSecondaryPin(other_trigger_switch);

  
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
  set_full_auto();

  // Set up the rev switch to toggle the flashlight
  rev_switch_button.set_pressed_command(&handle_flashlight);
  
}


float calculate_voltage() {
  //http://www.electroschematics.com/9351/arduino-digital-voltmeter/
  //R1 = 67750.0;  -see text!
  //R2 =  9970.0;  -see text!
   // Instead of dividing, 
  // used a voltimeter to measure at the analog read point, 
  // and then divided the actual bat voltage by the reading to get the multiplier,
  // which is then hardcoded in.
  float value = analogRead(voltimeter_pin);
  float vout = (value * 5.0) / 1024.0; // see text
  // Our reading was (vin/vout) = 15.42/1.955
  float vin = vout * 8.65;
  if (vin<0.09) {
    vin=0.0;//statement to quash undesired reading !
  }
  return vin;
  
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
  constexpr byte num_of_cells = 2;
  constexpr float battery_min_voltage = 3.4*num_of_cells;
  constexpr float battery_full_voltage = 4.2*num_of_cells;
  static int battery_percentage = 0;
  static short voltage_to_print = 0; // Doing this to avoid float operations every single display update when printing, probably not needed though.
  // First handle updating voltage reading if enough time has passed!
    if ( millis() - last_updated_voltage_at > 512 || last_updated_voltage_at == 0 ) {
      float voltage_to_disp = calculate_voltage();
      voltage_to_print = voltage_to_disp*100;
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

  // Per cell voltage guess
  display.print((voltage_to_print/num_of_cells)/100);
  display.print('.');
  byte decimal = (voltage_to_print/num_of_cells)%100;
  if ( decimal < 10 ) {
    display.print('0');
  }
  display.print(decimal);
  display.print('V');
  if ( pusher_was_stalled ) {
    display.print('S');
  }

  // Overall voltage
  display.setCursor(90, 2);
  display.print(voltage_to_print/100);
  display.print('.');
  decimal = voltage_to_print%100;
  if ( decimal < 10 ) {
    display.print('0');
  }
  display.print(decimal);
  display.print('V');
  
}

void render_feed_delay_modifier() {
  display.setCursor(1,display.height()-3*3);
  display.print("FDM:");
  display.print(feed_delay_modifier);
}

void render_flashlight_brightness() {
    display.setCursor(104, display.height()-3*3);
    display.setTextColor(1);
    display.setTextSize(1);

    // Check for the special cases of 1/2, which otherwise round down to 0!!!
    // While ugly to make special cases, uglier to add a decimal to all settings,
    // so this is what I choose.
    if ( current_flashlight_brightness == 1 ) {
      display.print(".4%");
    } else if ( current_flashlight_brightness == 2 ) {
      display.print(".8%");
    } else {
      display.print((current_flashlight_brightness*100)/255);
       display.print("% ");
    }

    // Begin flashlight drawing procedure!
    int x_offset = 89;
    int y_offset = 57;
    // Top of the flashlight
    display.drawPixel(x_offset,y_offset,1);
    display.drawPixel(x_offset+1,y_offset,1);
    display.drawPixel(x_offset+2,y_offset,1);
    display.drawPixel(x_offset+3,y_offset,1);
    display.drawPixel(x_offset+4,y_offset,1);
    display.drawPixel(x_offset+5,y_offset,1);
    display.drawPixel(x_offset+6,y_offset,1);

    // Bottom bar of flashlight
    display.drawPixel(x_offset,y_offset+2,1);
    display.drawPixel(x_offset+1,y_offset+2,1);
    display.drawPixel(x_offset+2,y_offset+2,1);
    display.drawPixel(x_offset+3,y_offset+2,1);
    display.drawPixel(x_offset+4,y_offset+2,1);
    display.drawPixel(x_offset+5,y_offset+2,1);
    display.drawPixel(x_offset+6,y_offset+2,1);

    // Flashlight toggle button
    display.drawPixel(x_offset+5,y_offset+1,1);

    // Flashlight butt 
    display.drawPixel(x_offset,y_offset+1,1);

    // Flashlight lens
    display.drawPixel(x_offset+7,y_offset-1,1);
    display.drawPixel(x_offset+7,y_offset+3,1);
    display.drawPixel(x_offset+8,y_offset-1,1);
    display.drawPixel(x_offset+8,y_offset+3,1);
    display.drawPixel(x_offset+8,y_offset,1);
    display.drawPixel(x_offset+8,y_offset+1,1);
    display.drawPixel(x_offset+8,y_offset+2,1);

    if (flashlight_status == HIGH) {
      // Flashlight rays
      display.drawPixel(x_offset+10,y_offset+1,1);
      display.drawPixel(x_offset+11,y_offset+1,1);
      display.drawPixel(x_offset+12,y_offset+1,1);
  
      display.drawPixel(x_offset+10,y_offset-1,1);
      display.drawPixel(x_offset+11,y_offset-1,1);
      display.drawPixel(x_offset+12,y_offset-1,1);
  
      display.drawPixel(x_offset+10,y_offset+3,1);
      display.drawPixel(x_offset+11,y_offset+3,1);
      display.drawPixel(x_offset+12,y_offset+3,1);
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
    render_flashlight_brightness();
    render_feed_delay_modifier();
    display.display();
  }
}

void retract_pusher_if_mag_out() {
  if (!magazine_in.query() && !cycler.query() && !pusher_was_stalled) {
    set_motor(true); // If mag is out, and pusher is extended, retract the motor. 
  }
}
void loop() {
  pusher_safety_shutoff();
  update_buttons();
  render_display();
  retract_pusher_if_mag_out();
}

