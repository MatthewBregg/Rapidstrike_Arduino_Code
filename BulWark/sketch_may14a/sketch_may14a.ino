////Core Blaster Controller Firmware
////Version 26.11 [DEVELOPMENTAL]
////by torukmakto4
////..DZ Industries
////This is part of
////PROJECT T19
////Forever Open - Forever Independent

////Project T19 and this Software (DZ Core) are released under the Creative Commons BY-NC 4.0 license.
////https://creativecommons.org/licenses/by-nc/4.0/
#include <Button_Debounce.h>

//state variables section
bool prevTrigState = 0;
bool currTrigState = 0;
unsigned long lastTriggerUp = 0;
bool firstRun = 1;

void selftest() {
  OCR1B = 290; //command flywheel drive torque
  OCR1A = 290;
  delay(180); //for a brief blip
  OCR1B = 230; //and then shut them down
  OCR1A = 230;
}



////PWM Timer1 interrupt governor control variables
volatile int gov_currentBit;               //Loop counter/index (shall not be unsigned)
volatile unsigned long save_ocr1a;         //Save throttle settings to resume emitting them after all digital speed bits are shifted out
volatile unsigned long save_ocr1b;
volatile unsigned long governor;           //this has to be a volatile even though the ISR does not modify it
volatile boolean gov_enable_strobe = false; //Set to true to fire off update
volatile boolean gov_packet_active = false; //True while shifting bits out in the gov ISR, resets at the end of the cleanup cycle
////FlyShot constant/setup section
volatile unsigned long gov_ocr_t0h = 26;   //T0H = 100us (Less than 250us)
volatile unsigned long gov_ocr_t1h = 90;   //T1H = 400us (More than 250 and less than MIN_RC_PULS)
//TL is by the PWM period. We will just keep running at 400Hz.

int gov_update_repeats = 10;               //Number of min. times to repeat (Like Dshot commands, a one-time setting should be sent about 10 times to ensure noise robustness, etc.)

void enableGovernorInterrupt() {
  cli();
  TIMSK1 = 0b00000010; //set OCIE1A
  sei();
}

void disableGovernorInterrupt() {
  cli();
  TIMSK1 = 0b00000000; //clear OCIE1A to disable
  sei();
}

boolean setGovernorBoth(void) {
  if (governor > 0x7fff) {
    return false; //Die: Cannot set governor to more than 0x7fff
  }
  governor |= 0x8000;                             //Set msb (flag bit: this is a governor update frame) to 1 (n.b.: 7fff becomes ffff)
  gov_currentBit = 15;                            //Prepare for enabling interrupt routine
  save_ocr1a = OCR1A;                             //Save throttle command (restored by ISR during the cleanup cycle after packet is over)
  save_ocr1b = OCR1B;
  enableGovernorInterrupt();                      //Enable ISR
  delay(10);                                      //A few idle cycles (timer is still creating throttle pulses)
  gov_enable_strobe = true;                       //Push the start button
  while ((gov_enable_strobe || gov_packet_active)) {
    delayMicroseconds(1000);                     //Either the strobe state hasn't been checked yet or the packet is still transmitting: block until done
  }
  delay(10);                                       //Ensure some clean throttle pulses get fired between governor packets so drive doesn't disarm/balk
  disableGovernorInterrupt();                      //Mute ISR
  return true;
}


const long motorPolepairs = 7;         //Pole order of your flywheel motors
volatile unsigned long speedSetpoint;     //us (6 * TIMING_MAX / 4)
unsigned long speedOffsetMargin;          //Consider tach in range if speedOffsetMargin us greater than (lower speed) speedSetpoint
unsigned long speedOffsetMarginMin = 30;  //At low speed   (This compensates competing effects from period being 1/f, control loop performance, and tach resolution so isn't much more
unsigned long speedOffsetMarginMax = 20;  //At max speed    than a fudge factor)
unsigned long minRPM = 3000;              //Fly speed command min (Set this appropriately for your system to ensure passing darts at minimum speed)
unsigned long maxRPM = 25511;             //Fly speed command max (Set this appropriately. If the drive can't actually reach this speed you won't be able to fire!)
void updateSpeedFixed(const long setpointRPM) {
  // For checksumming purposes, push the governor twice!
  // The first push sets dib_h/l_old, and the second push sets dib_h/l
  // and then the governor itself!
  for ( int i = 0; i != 2; ++i ) {
    //Set setpointRPM from limitRPM only, update governor, push governor update, and update tach control parameters.
    const long setpointGovernor = (320000000 / (setpointRPM * motorPolepairs)); //Convert to governor, which is 8 * TIMING_MAX (i.e. TIMING_MAX * CPU_MHZ (=16) / 2) at full resolution
    //Push governor update
    governor = setpointGovernor;
    while (!setGovernorBoth()) {}
  }
}

////2 channel Governor config ISR (Easy enough to make 2 independent ones for apps that need 2 independent speed controlled drives)

ISR(TIMER1_COMPA_vect) {
  //Timer1 channel A compare match: Fires when hitting OCR1A (which is the falling edge of a PWM high period that started at overflow)
  //We have from the compare match until at least gov_ocr_t0h (shortest pulsewidth) ticks after overflow (the next compare match) to calculate and update OCR1x.

  if (gov_enable_strobe) {
    gov_packet_active = true;
    if ((gov_currentBit + 1) > 0) {                             //Start shifting out packet, MSB first
      if (governor & (0x0001 << gov_currentBit)) {            //Mask the packet with a 1 LSLed i times to select the desired bit (start at 15)
        OCR1A = gov_ocr_t1h;                                 //T1H
        OCR1B = gov_ocr_t1h;
      } else {                                               //else: is 0
        OCR1A = gov_ocr_t0h;                                 //T0H
        OCR1B = gov_ocr_t0h;
      }                                                      //Nb: T0L/T1L is just the remainder of the period at 400Hz
      gov_currentBit--;                                      //ends after running with gov_currentBit = 0, thus leaving it at -1
    } else {                                                    //last data cycle has run through with gov_currentBit at 0, then decremented it to -1 after - do cleanup
      OCR1A = save_ocr1a;                                     //restore saved throttles (important to do this NOW!! before the next PWM cycle needs them!)
      OCR1B = save_ocr1b;
      //TIMSK1 = 0b00000000;                                    //clear OCIE1A to disable this ISR (A self-terminating ISR is badwrongevil, it seems?)
      //bitSet(TIFR1, 1);                                       //Clear pending interrupt (?)
      gov_packet_active = false;                              //Active packet flag to false
      gov_enable_strobe = false;                              //Disable
    }
  }
  //Here is where the inactive state lands when this ISR is on and waiting for gov_enable_strobe.
  //Do nothing. Don't mod anything because foreground code is probably loading variables right now.
}

float calculate_voltage() {
  //http://www.electroschematics.com/9351/arduino-digital-voltmeter/
  constexpr double  R1 = 68200.0; // -see text!
  constexpr double R2 =  9850.0; // -see text!
  constexpr double multiplier = (R2 / (R1 + R2));
  // Instead of dividing,
  // used a voltimeter to measure at the analog read point,
  // and then divided the actual bat voltage by the reading to get the multiplier,
  // which is then hardcoded in.
  float value = analogRead(A5);
  float vout = (value * 5.0) / 1024.0; // see text
  float vin = vout / multiplier; // 8.65;
  if (vin < 0.09) {
    vin = 0.0; //statement to quash undesired reading !
  }

  return vin;

}



float get_motor_speed_factor(float volts) {
  float value = volts / ((calculate_voltage() + calculate_voltage()) / 2.0);
  if ( value < 1 ) {
    return value;
  }
  return 1;

}


BasicDebounce trigger = BasicDebounce(12, 5);
// Cycle.query() is true when the pusher is retracted.
BasicDebounce cycle = BasicDebounce(4, 3);
// Use this switch to stop revving the flywheels when we are no longer firing and the pusher retracts.
// Another option: When trigger is released, stop flywheels on retract, start on release (pusher overshoot).
// The downside of this? There's the (small) risk of rapidly revving/unreving the flywheels while they are locked,
// defeating even brushless motors robust stall protection.
// I suppose this could be mitigated by putting a limit on how often we rev the flywheels but that creates complicated software fast.
// Perhaps using this switch, but with just a 10ms debounce would be a good compromise?
BasicDebounce cycle_debounced = BasicDebounce(4, 10);

void setup() {

  //pin 11 and 12 trigger switch
  //11 outside connector pin (default high)
  //12 inside connector pin (default low)
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  //pin 9/10 flywheel motor controller PWM throttle signal
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //pin 3: Pusher Motor PWM
  pinMode(3, OUTPUT);
  // Set the pusher motor to maintain roughly 12 volts.
  analogWrite(3, 255.0 * get_motor_speed_factor(12));
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1B = 230; //write 920us low throttle

  // Set the pusher motor PWM frequency to be 3.9K
  // http://ceezblog.info/2018/07/10/arduino-timer-pwm-cheat-sheet/
  TCCR2B = (TCCR2B & B11111000) | B00000010;


  // pin 4: RElay
  pinMode(6, OUTPUT);
  //pin 4/5: bolt limit switch (pullup)
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  pinMode(A5, INPUT);
  // We use A3 as a gnd.
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);
  // We use 8 as VCC
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);


  cycle.AddSecondaryPin(5);
  cycle_debounced.AddSecondaryPin(5);
  trigger.AddSecondaryPin(11);
  cycle.set_pressed_command(&pusher_retracted);
  cycle.set_released_command(&pusher_extended);

  Serial.begin(9600);
  Serial.println("Voltage");
  Serial.println(calculate_voltage());

}

void update_buttons() {
  trigger.update();
  cycle.update();
  cycle_debounced.update();
}

void first_run() {
  ////continue startup
  selftest();
  // Set the Flywheel Governor to RPM.
  for ( int i = 0; i != 10; ++i ) {
    // Set the speed 10 times for paranoia reasons!
    updateSpeedFixed(37000);
  }

  //clear flag
  firstRun = false;
}

bool pushing = false;
const long pusher_timeout = 300;
bool pusher_stalled = false;
long pusher_millis = 0;

void pusher_retracted(BasicDebounce* ) {
  pusher_millis = millis();
}

void pusher_extended(BasicDebounce*) {
  pusher_millis = millis();

}
void set_pusher(bool on) {

  if (on && !pusher_stalled) {
    if (!pushing) {
      pusher_millis = millis();
    }
    pushing = true;
    digitalWrite(6, HIGH);
    // analogWrite(3, 255.0 * .70);
    //analogWrite(3, 255.0 * .65);
  } else {
    digitalWrite(6, LOW);
    pushing = false;
  }
}

void handle_pusher_stalls() {
  if ( pushing && ((millis() - pusher_millis) > pusher_timeout)) {
    // Stall!
    pusher_stalled = true;
    set_pusher(false);
    shutoff_flywheels();
  }
}
bool revving = false;
long last_turned_down_flywheels = 0;
void shutoff_flywheels() {
  if (revving) {
    last_turned_down_flywheels = millis();
  }
  revving = false;
  OCR1B = 230; //shutdown
  OCR1A = 230;
}
void rev_flywheels() {
  revving = true;
  OCR1B = 500; //go
  OCR1A = 500;
}
void upkeep() {
  update_buttons();
  handle_pusher_stalls();
}
void delay_and_upkeep(int delay) {
  int init = millis();
  while ((millis() - init) < delay) {
    upkeep();
  }

}

// Rev the flywheels, handle the FD
void InitFiring() {
    const int FD_STAGE_1 = 3000;
    const int FD_STAGE_2 = 1000;
    const long millis_since_rev = millis() - last_turned_down_flywheels;
    // Meh though it may be, looks like we are starting with just 118 FPS...
    // Rev
    // 105 FD = GOOD!
    // 50: Some bad shots but actually seems good overall?!
    // Lets go with 65 to be sure?
    // 20: Confirmed too low. 
    // So somewhere between 20-50, I could spend ages microoptimizing over 30 MS, likely somewhere in 15 MS saved, or I could call it a day + play it safe at 65.
    // 50 Already gave me 1 or 2 lowish readings, so I think 65 is p good.
    // Final FD: 65!!
    // Onto stage 2!
    // Within 2k seconds 30 MS rev gives a binary distribution of 127/108?! Unknown.... Average before was 105. Let's try cranking it up a bit?
    // Tried 2k delay and 60 MS rev, same results, I'm calling it good.
    // 3K/30 ms rev confirmed good. The wheels stop at 4 seconds in.
    // I could from here either: Decrease the rev time, or go all the way out to 4 seconds, but honestly? I'm good. Let's move on. 
    // Plus, we don't want to live on the bleeding edge here, this blaster has enough moving parts, complexity, and finicky bits. 
    // Stage 1.5: Can we get away with 0 rev at 1 S?
    // Yup. That's our FD!

    /* 
    rev_flywheels();
    delay(500); // Fully rev
    shutoff_flywheels(); // Cool off
    delay(1000);
    rev_flywheels();
     // No re-rev!!
    return;
    */

    rev_flywheels();
    // Delay
    if (millis_since_rev < FD_STAGE_2  ) {
     
      delay(15); // FD_STAGE_2 delay, revved within FD_STAGE_2 MS 
     
    } else if (millis_since_rev < FD_STAGE_1 ) {
          
            delay(30); // FD_STAGE_1 delay, revved within FD_STAGE_1 ms
    } else {
           
      delay(65); // Full Feed Delay. Have not revved recently. 
    }
}
void loop() {
  // Switch to a don't block the loop approach, import my debounce library and let's do this correctly.
  if (firstRun) {
    first_run();
  }

  upkeep();



  if (trigger.query()) {

    // Initialize pusher if we otherwise weren't pushing
    // If we are already pushing, then this isn't our first time in this loop for this trigger down.
    bool first = !pushing;

    // Rev flywheels and handle FD
    if (first) {
      pusher_stalled = false;
      InitFiring();
    }

    // Initialize the pusher.
    set_pusher(true);

    // For the first cycle, make sure we fire one dart by waiting for the
    // pusher to extend.
    while (first && cycle_debounced.query()) {
      upkeep();
    }

  } else {
    if ( cycle.query() ) {
      set_pusher(false);
    }
    // set_pusher(!cycle.query());
    if (cycle_debounced.query()) {
      shutoff_flywheels();
    }
  }
}

