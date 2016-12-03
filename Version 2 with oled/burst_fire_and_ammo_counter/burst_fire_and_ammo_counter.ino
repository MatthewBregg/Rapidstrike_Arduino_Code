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

/* Uncomment this block to use hardware SPI
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);
*/

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 


#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
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
  delay(2000);
  display.setRotation(2); //rotate display
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

  // Volt meter
  //---------------------
   pinMode(A3, INPUT);
  //---------------------

  
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
float voltage_to_disp = 0.0;
unsigned long last_updated_voltage_at = 0;
bool motor = false;
void loop() {
  // put your main code here, to run repeatedly:
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  //Print pusher status
  display.print("Motor enabled? ");
  display.println(motor);
  //Print switches status
  display.print("Rev Switch ");
  display.println(digitalRead(rev_switch));
  display.print("Trigger Switch ");
  display.println(digitalRead(trigger_switch));
  display.print("Cycle Switch ");
  display.println(digitalRead(cycle_switch));
  display.print("mag Switch ");
  display.println(digitalRead(mag_switch));
  display.print("selA Switch ");
  display.println(digitalRead(selector_switch_a));
  display.print("selB Switch ");
  display.println(digitalRead(selector_switch_b));
  
  
  //Print voltage also, only update voltage on an fixed interval to avoid flicker
  if ( millis() - last_updated_voltage_at > 512 || last_updated_voltage_at == 0 ) {
    voltage_to_disp = calculate_voltage();
    last_updated_voltage_at = millis();
    }
  display.print("Voltage is ");
  display.println(voltage_to_disp);
  display.display();
  //set_motor(motor);
  delay(20);
  //motor = !motor;
  digitalWrite(nchan_flywheel_mosfet_pin,!digitalRead(rev_switch));

}

