#include <Button_Debounce.h>
#include <Servo.h>
 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* OLED Display preprocessor */

// If using software SPI (the default case):
#define OLED_MOSI  7 /// Black
#define OLED_CLK   6 // Yellow
#define OLED_DC    9 // Pink
#define OLED_CS    12 // RED
#define OLED_RESET 8 // Purple
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
constexpr int motor_pin = 2;


//These switches are active_low, aka, switch closed == low.
constexpr int trigger_switch = 4; // A1 is connected to NC of trigger switch, 4 to NO
constexpr int cycle_switch = 5;
constexpr int rev_switch = 3;
constexpr int mag_switch = A5;
constexpr int voltimeter_pin = A7;
constexpr int selector_switch_a = A2;
constexpr int selector_switch_b = A3;
constexpr int flashlight_pin = 0; 
/* End the section on pins */



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

BasicDebounce trigger = BasicDebounce(trigger_switch, 20, LOW);
BasicDebounce rev = BasicDebounce(rev_switch, 5, LOW);
// Idea, perhaps have cycler, and ammo cycler objects.
// cycler would have a very low debounce delay, 
// and be used for fire control where bouncing might have to happen for good cycle control.
// Ammo cycler can have a high debounce, and be used for ammo counting.
BasicDebounce cycler = BasicDebounce(cycle_switch,5, LOW);
BasicDebounce magazine_in = BasicDebounce(mag_switch,50, LOW);
BasicDebounce selector_a = BasicDebounce(selector_switch_a,50, LOW);
BasicDebounce selector_b = BasicDebounce(selector_switch_b,50, LOW);


void enable_flywheels(bool rev) {
    if(rev){
      //start flywheels
      OCR1B = 500; //go
  } else {
    OCR1B = 230; //shutdown
  }
}
bool ignore_rev_trigger = false;
void handle_rev_trigger() {
  if ( ignore_rev_trigger ) {
    return;
  }
  enable_flywheels(rev.query());
}



void block_until_revved() {
  constexpr byte feed_delay = 150;
  if ( rev.query() ) {
    // IF we are already revving with the rev trigger, then skip this logic and thus fire immedietely.
    return;
  }
  ignore_rev_trigger = true; // Handling revving for the user now.
  enable_flywheels(true);
  delay(feed_delay);
  return;
}

void stop_flywheels_if_not_revving() {
  ignore_rev_trigger = false; // Done firing, returning rev control to user.
  if (!rev.query()) {
    enable_flywheels(false);
  }
}

void set_fire_mode() {
 if ( motor_enabled ) {
  return; // If we set the firing mode while the motor is running, shots_to_fire gets reset, and that's bad!
  // We could probably be fine to remove that, but changing fire modes mid shot is probably a bad idea anyway.
 }
 if ( !selector_a.query() && !selector_b.query() ) {
  set_burst_fire(3);
 } else if ( selector_a.query() ) {
    set_full_auto();
 } else if ( selector_b.query() ) {
    set_burst_fire(1);
 }
}

void update_buttons() {
  trigger.update();
  rev.update();
  cycler.update();
  magazine_in.update();
  selector_a.update();
  selector_b.update();
  set_fire_mode();
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
     shots_to_fire = min(shots_to_fire+burst_mode,max_stack); // Add burst_mode shots to be fired (or as firable if in fdl style)
     // Do not allow stacking more shots than max_stack.
     block_until_revved();
     set_motor(true);
  }
}

void full_auto_trig_press_handler(BasicDebounce* button) {
  block_until_revved();
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



void handle_pusher_retract(BasicDebounce* button) {
  // Keep track of shots to fire and ammo count
  ++shots_fired;
  --shots_to_fire;
  if ( shots_to_fire <  0 ) {
    shots_to_fire = 0;
  }
  
  // Disable pusher if we hit our burst limit done burst firing
  if ( shots_to_fire  == 0 && fire_mode == burst_fire ) {
    set_motor(false);
    stop_flywheels_if_not_revving();
  }

  // Also allow stopping a burst early. 
  // Add a check to only run this when mode is full auto if that behavior isn't desired.
  if ( !trigger.query() ) {
    shots_to_fire = 0;
    set_motor(false);
    stop_flywheels_if_not_revving();
    
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

  // Set up flywheels and their ESCs
  //pin 10 flywheel motor controller PWM throttle signal
  pinMode(10, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1B = 230; //write 920us low throttle


  // Motor controls
  //---------------------------------------
  pinMode(motor_pin, OUTPUT); //Motor
  digitalWrite(motor_pin, LOW);  

  // Set up the two pin switches for h/w debouncing
  cycler.AddSecondaryPin(A4);

  //---------------------------------------
  // Flash light
  pinMode(flashlight_pin,OUTPUT);

  // Volt meter
  //---------------------
   pinMode(voltimeter_pin, INPUT);
  //---------------------


  // Set up the cycle handler
  cycler.set_pressed_command(&handle_pusher_retract);

  // Set up trigger buttons & firing mode, default to 2 shot burst.
  set_burst_fire(2);
  
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
  // Our reading was (vin/vout) = 10.45/1.332
  float vin = vout * 7.85;
  if (vin<0.09) {
    vin=0.0;//statement to quash undesired reading !
  }
  return vin;
}



unsigned long last_updated_voltage_at = 0;

void render_ammo_counter() {
    constexpr byte shiftAmmoCounterLeft = 10;
    if ( !magazine_in.query()) {
      // Can either fill the circle, draw an X, or display something like C.O. in circle. (Or graphic for mag out, but that's above me. 
      // I like the X idea best, simple, and easy to understand meaning.
      display.drawLine((display.width()/2-19)+shiftAmmoCounterLeft,(display.height()/2-19),(display.width()/2+19)+shiftAmmoCounterLeft, display.height()/2+19, WHITE);
      display.drawLine((display.width()/2+19)+shiftAmmoCounterLeft,(display.height()/2-19),(display.width()/2-19)+shiftAmmoCounterLeft, display.height()/2+19, WHITE);
      shots_fired = 0;
    } else {
      // Font width is 5, height is 8, * scale factor.
      const uint8_t scale_factor = 4;
      const uint8_t font_width = 5*scale_factor;
      const uint8_t font_height = 8*scale_factor;
      const char tens_place = '0'+shots_fired/10;
      const char ones_place = '0'+shots_fired%10;
      display.drawChar((display.width()/2-(font_width)-1)+shiftAmmoCounterLeft, display.height()/2-(font_height/2)+2,tens_place,1,0,scale_factor); // Take the center of the screen, and shift over enough for 2 chars.
      display.drawChar((display.width()/2+3)+shiftAmmoCounterLeft, display.height()/2-(font_height/2)+2,ones_place,1,0,scale_factor); // Take the center of the screen, and shift over enough for 1 chars.
    }
     display.drawCircle(display.width()/2+shiftAmmoCounterLeft,display.height()/2,31, WHITE);
}


void render_battery_indicator() {
  const float battery_min_voltage = 9;
  const float battery_full_voltage = 12.6;
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
 // display.print(voltage_to_disp);
  display.print(voltage_to_print/100);
  display.print('.');
  display.print(voltage_to_print%100);
  display.print('V');
  if ( pusher_was_stalled ) {
    display.print('S');
  }
  if ( trigger.query() ) {
    display.print('T');
  }
  if (rev.query() ) {
    display.print('R');
  }

  if ( digitalRead(rev_switch) == LOW ) {
    display.print('Y');
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


void loop() {
  pusher_safety_shutoff();
  handle_rev_trigger();
  update_buttons();
  render_display();
  retract_pusher_if_mag_out();
}
