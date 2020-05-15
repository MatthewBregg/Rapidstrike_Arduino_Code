 ////Core Blaster Controller Firmware
 ////Version 26.11 [DEVELOPMENTAL]
 ////by torukmakto4
 ////..DZ Industries
 ////This is part of
 ////PROJECT T19
 ////Forever Open - Forever Independent

 ////Project T19 and this Software (DZ Core) are released under the Creative Commons BY-NC 4.0 license.
 ////https://creativecommons.org/licenses/by-nc/4.0/
 
 ////This targets:
 ////Core Backplane
 ////processor card - Arduino Pro Mini 5V/16MHz - ATMega328P
 ////bolt driver card - DRV8825 carrier
 ////200s/rev 2 phase hybrid stepper
 ////direct drive scotch yoke bolt, 1 shot per rev
 ////SPST bolt limit switch
 ////SPDT trigger switch
 
 ////Features:
 ////Intelligent single-trigger blaster control.
 ////Fully configurable STC parameters. Adaptable to any user and hardware kinetics beyond its native blaster.
 ////3-step feed delay scheduling.
 ////Stepper motor bolt drive with true linear velocity ramps and position error auto-recovery.
 ////Power-On Selftest and bolt reset.
 ////Runtime-selectable alternate preset mode - configured stock as "turbo mode" with higher ROF.
 ////400Hz hardware PWM flywheel control.

 ////There is NOT:
 ////Selective fire. That's mostly for people with shitty trigger logic.
 ////Flywheel variable throttle - T-Series blasters have closed loop flywheel drives where speed is configured elsewhere. 
 ////ROF live-adjustment, aside from turbo mode.
 ////Really any live-adjustments, or any physical controls except the trigger. You turn it on, you shoot it, and you turn it off.
 
 ////TBD: make fire() aware of the limit switch and reset its position index OTF on switch low - avoid any "RS style" trailer shots as some cases give,
 ////     when some steps are lost on a sticky round and then we're coming out of fire() with the bolt in an unexpected position.
 ////     OTF bolt stall mitigation rather than on trigger-up - if too many skipped steps are detected while firing (from the above), reverse fast and restrike.
 ////     Adaptively adjust bolt speed based on step loss events - perhaps have an adaptive bolt speed mode in which we learn the gun's limits during use,
 ////     and save that to EEPROM.

 ////changelog of the new era:
 ////04-28-18 - adjust delay parameters - for possible startup weirdness, and more reliable full velocity
 ////           add new speedier bolt drive commutate()
 ////           increase length of selftest wheel drive blip for reliable rotation with aforementioned startup weirdness (component variation)
 ////           implement turbo mode - hold trigger down when booting to activate
 ////05-01-18 - implement 3 stage feed delay. Tweak base feed delay to better account for nonideal flywheel drive tuning and hardware.
 ////           Better comment feed delay scheduling strategy and tuning.
 ////           Take a crack at running turbo mode at 14.2rps since the 13.8 was VERY solid.
 ////           Give user "knob" variables for the recent shot window and new driveStarted window, and dissociate recent shot window from bolt motor current shutoff.
 ////           Give a user "knob" variable for the bolt motor current shutoff delay after firing.
 ////05-04-18 - Adjust feed delay schedules - bump up base for even more reliable 100% velocity first shots.
 ////           Increase driveCoastTimes to account for real Hy-Con parts and recentShotTimes to account for hard closed-loop accels at high rpm,
 ////           and make both corresponding reductions more aggressive.
 ////           Decrease Turbo ROF to 13.8 due to some stalling on sticky old darts. (Can you tell the difference between 14.2 and 13.8?)
 ////           Selftest blip wheel motors with low duty, no need make noise and waste energy.
 ////11-29-18 - Add license information to file.
 ////           Update version to 26.11
 
 ////This uses the per-cycle lightweight calculation of stepper commutation period described in
 ////http://hwml.com/LeibRamp.pdf
 
 ////This is free and open source software.
 ////Eywa has been made aware of this fact.
 ////You will not get far, should you try to commercialize it.
 

 //FEED DELAY PARAMETERS
 //The Core single-trigger control delay schedule is implemented as 5 distinct values:
 //  * A base feed delay
 //  * 2 compensation values (delay REDUCTIONS) for the main conditions that warrant LESS feed delay:
 //    * The drives are turning at quite high speed after a recent shot - reaccel to setpoint will be very fast
 //    * The drives are turning at sufficient speed that rotor angle is known and current will be applied immediately, but most of the acceleration remains to be done
 //  * 2 time windows that predict these conditions.
 //
 //****** The compensations DO NOT stack, EITHER ONE of them is applied to feedDelayBase at a time.
 //
 //Adjust feedDelayBase:
 //  Down if you have overly laggy cold-start shot lock time.
 //  Up if you have low or inconsistent cold-start shot velocity.
 //Adjust recentShotCompensation:
 //  Up (reduces delay) if you want snappier followup shots within recentShotTime.
 //  Down (increases delay) if inconsistent/low followups, or if you find that lock time is too variable from cold-start to rapid followup.
 //Adjust driveStartedCompensation:
 //  Up (reduces delay) for snappier shots when wheels are still turning but after recentShotTime is over.
 //  Down (increases delay) if low/inconsistent shots under this condition.
 //Adjust recentShotTime:
 //  Up to make recentShotCompensation apply for a longer window after firing, down for shorter. (This depends on your recentShotCompensation value what is appropriate.)
 //Adjust driveCoastTime:
 //  Up if you find feedDelayBase is being enforced when the wheels are still consistently turning/motors being tracked
 //  (i.e. no red desync warning lights will have flashed on the controllers at the end of coastdown, if your controllers have LEDs supported by SimonK)
 //  Down if you find that the drives can get caught cold-starting the motors without being given the full feedDelayBase to start AND accelerate.
 //
 //Additionally: There are duplicates of all these parameters that are separately configurable for turbo mode.
 //This is not only a function of the bolt speed being different, thus mechanically shifting the delays,
 //but of the expected difference of use-case for turbo mode.
 //How you doctor your delay schedules for turbo mode is up to how you use turbo mode.
 // 
 //Restrictions:
 //driveStartedCompensation and recentShotCompensation MUST be less than or equal to feedDelayBase (if equal, this commands a feed delay of 0 for that circumstance)
 //driveStartedCompensation should be less than recentShotCompensation to make any sense (inverse causes delay to decrease with increasing time).
 //recentShotTime must be less than driveCoastTime (if not, then recentShotCompensation is never used and we jump straight to driveStartedCompensation).
 //
 //Errata/Oddities:
 //Since the Prototype T19/Model Pandora days and "T18V21"/"T18V22"/Core 24 firmwares, base delay has increased. This is because the bolt(ROF) keeps getting faster,
 //and thus the time length of the ~1/3 cycle between the bolt motion command and the actual dart contact with wheels keeps decreasing. Also,
 //it is because I am trying to get vanilla settings that make most guns run at full velocity "out of the box", whereas I may have cut it very close
 //on my personal builds like the Prototype. And contending with occasional motor controller misbehaviors too.

 
 int feedDelayBase = 90; //ms
 int feedDelayBaseTurbo = 90; //ms - above when turbo mode is active
 int driveStartedCompensation = 30; //ms - delay reduction when drives ought to be still spinning, and rotor angles being tracked
 int driveStartedCompensationTurbo = 30; //ms - above when turbo mode is active 
 int recentShotCompensation = 50; //ms - delay reduction when flywheel speed ought to be "considerable"
 int recentShotCompensationTurbo = 50; //ms - above when turbo mode is active
 int recentShotTime = 1700; //ms - time window after shot in which drive speed ought to be "considerable"
 int recentShotTimeTurbo = 1700; //ms - above when turbo mode is active
 int driveCoastTime = 3000; //ms - time window after shot in which drives should have enough speed for BEMF-trackable rotor angle
 int driveCoastTimeTurbo = 3000; //ms - above when turbo mode is active
 
 
 //BOLT DRIVE PARAMETERS - don't mess with most of these unless you know what you are doing, EXCEPT:
 //runSpeed and runSpeedTurbo are your "knobs" for ROF in normal and turbo mode resp. (Note: ROF in rps = 1250/(speed value))
 unsigned long startSpeed = 400; //us (default 400 - leave alone)
 unsigned long shiftSpeed = 150; //us (default 150)
 unsigned long runSpeed   = 110; //us (default 110 - 125 for reliable operation on 3S)
 unsigned long runSpeedTurbo = 90; //us - turbo mode
 double accelPhaseOne = 0.000000253494; //m value for ramp in low speed region
 double accelPhaseTwo = 0.000000180748; //m value for ramp in high speed region
 double decel =         0.000000439063; //m value for hard decel
 int boltShutdownTime = 1000; //ms - post-shot delay after which motor current is turned off
 
 //state variables section
 bool prevTrigState = 0;
 bool currTrigState = 0;
 unsigned long lastTriggerUp = 0;
 int delayReduction = 0;
 int stepsToGo;
 bool firstRun = 1;
 bool boltHomed = 0;
 double currSpeed = startSpeed; //to be used by fire() to be aware of motor speed from last run and mate speed ramp to that
 double stepdelay; //us
 int selftestStepCounter = 0;
 int selftestCycleCounter = 0;
 int turboMode = 0; //flag for mostly future use
 
 void commutate(double commDelay){
    //function to commutate the stepper once. note- immediate rising edge, trailing delay    
    //digitalWrite(3, HIGH);
    PORTD |= 0b00001000;
    delayMicroseconds((unsigned long)(commDelay/2));
    //digitalWrite(3, LOW);
    PORTD = PORTD & 0b11110111;
    delayMicroseconds((unsigned long)(commDelay/2));
} 
 
 bool decelerateBoltToSwitch(){
  //try to gracefully end drivetrain rotation
  //called after the last fire() has returned
  //return true for home and false for not home
  
  //fire() runs the bolt forward 720 clicks (on 4:1 mode) leaving us 80 to bring it down to ~3rps where we know it can stop instantly and keep position.
  //abort instantly on limit switch low
  stepsToGo = 150;
  stepdelay = currSpeed;
  while((stepsToGo > 0) && (PIND & 0b00010000)){
    commutate(stepdelay);
    if(stepdelay<startSpeed) {stepdelay = stepdelay*(1+decel*stepdelay*stepdelay);}
    stepsToGo--;
  }  
  currSpeed = startSpeed;
  boltHomed = 1;
  return !(PIND & 0b00010000);
}

bool reverseBoltToSwitch(){
  //this is called if decelerateBoltToSwitch() returns false
  stepsToGo = 800; //up to a full rev permitted (TBD)
  //set bolt direction reverse
  digitalWrite(2, HIGH);
  //run bolt back at idle speed
  while((PIND & 0b00010000)){
    commutate(startSpeed);
    stepsToGo--;
    if(stepsToGo == 0){
      //ran out of angle, die and reset direction
      digitalWrite(2, LOW);
      return false;
    }
  }
  digitalWrite(2, LOW);
  return !(PIND & 0b00010000);
}

void selftest(){
  //somewhat of a placeholder now without much fault detection capability in the hardware... YET.
  //Still does tactile confirmation of the proper operation of all motor drives.
  //You should get when booting up:
  //Bolt motor: Thunk
  //Flywheel motors (SimonK drive startup): f1 f2 f3, f4f4f4
  //Bolt motor: GRRRR (or GRRRR GRRRR if turbo mode on)
  //Flywheel motors: Vroom! (Both wheels must turn)
  
  delay(500);
  //Make the bolt motor rumble to confirm it has power/works.
  //note bolt motor current should have been already turned on by the calling function, just like the other bolt-related functions
  selftestCycleCounter = 0;
  while(selftestCycleCounter < 3){
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(2, LOW);
    while(selftestStepCounter < 4){
      commutate(4500);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(2, HIGH);
    while(selftestStepCounter < 4){
      commutate(4500);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  if(turboMode){
    //do a second rumble when turbo active for user feedback
    delay(100);
     selftestCycleCounter = 0;
     while(selftestCycleCounter < 3){
     //do 4 microsteps one direction
     selftestStepCounter = 0;
     digitalWrite(2, LOW);
     while(selftestStepCounter < 4){
       commutate(4500);
       selftestStepCounter++;
     }
     //and do 4 microsteps back the other way
     selftestStepCounter = 0;
     digitalWrite(2, HIGH);
     while(selftestStepCounter < 4){
       commutate(4500);
       selftestStepCounter++;
     }
    selftestCycleCounter++;
    }
  }
  digitalWrite(2, LOW);
  delay(80);
  OCR1B = 290; //command flywheel drive torque
  delay(180); //for a brief blip
  OCR1B = 230; //and then shut them down
}
  
 

void fire(){
  //loop called to fire a shot
  
  //set distance to run bolt forward
  stepsToGo = 720;
  //if continuing a previous instance add 80 steps
  if(!boltHomed) {stepsToGo += 80;}
  
  //get start point for first ramp
  if(currSpeed < startSpeed){
    //bolt already running
    stepdelay = currSpeed;
  } else {
    //bolt not running
    stepdelay = startSpeed;
  }
  // do first ramp if speed below shiftpoint
  while(stepdelay > shiftSpeed){
      commutate(stepdelay); 
      stepdelay = stepdelay*(1-accelPhaseOne*stepdelay*stepdelay);
      stepsToGo--;
  }
  //do second speed ramp if speed above shift but below running speed
  while(stepdelay > runSpeed){
    commutate(stepdelay);
    stepdelay = stepdelay*(1-accelPhaseTwo*stepdelay*stepdelay);
    stepsToGo--;
  }
  //do constant speed run until out of steps
  while(stepsToGo > 0){
    commutate(stepdelay);
    stepsToGo--;
  }
  currSpeed = stepdelay;
  boltHomed = 0;
}

void setup(){
 
  //pin 11 and 12 trigger switch
  //11 outside connector pin (default high)
  //12 inside connector pin (default low)
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
 
  //pin 10 flywheel motor controller PWM throttle signal
  pinMode(10, OUTPUT);
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1B = 230; //write 920us low throttle
  
  //bolt drive hardware
  //DRV8825 stepper driver card
  //pin 2: direction
  pinMode(2, OUTPUT);
  //drive low
  digitalWrite(2, LOW);
  //pin 3: step
  pinMode(3, OUTPUT);
  //pin 4: bolt limit switch (pullup)
  pinMode(4, INPUT);
  //pin 5,6,7: microstep mode select M2,M1,M0
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  //pin 8: enable
  pinMode(8, OUTPUT);
  //turn motor current off
  digitalWrite(8, HIGH);
  
  //configure stepper driver parameters
  //test config 4:1 microstep. TBD
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
 
}

void loop(){
  if(firstRun) {
    //catch trigger down at boot (turbo mode requested)    
    if(digitalRead(11) && !digitalRead(12)) {
      //enable turbo mode
      //set flag
      turboMode = 1;
      //
      //bolt run speed
      runSpeed = runSpeedTurbo;
      //feed delay base
      feedDelayBase = feedDelayBaseTurbo;
      //recent shot comp
      recentShotCompensation = recentShotCompensationTurbo;
      //drive-started comp
      driveStartedCompensation = driveStartedCompensationTurbo;
      //recent shot time
      recentShotTime = recentShotTimeTurbo;
      //drive coast time
      driveCoastTime = driveCoastTimeTurbo;
      //trap loop - hangs if trigger down, requires trigger release to continue boot and prevents firing shots
      //Note: This also idiotproofs if someone plugs the trigger cable in backwards causing a constant trigger-down input.
      while(digitalRead(11) && !digitalRead(12)) {delay(5);}
    }
    ////continue startup
    //do selftest and then force bolt home from unknown state
    //turn bolt motor current on
    digitalWrite(8, LOW);
    //avoid undefined 8825 behavior
    delay(20);
    //do selftest routine
    selftest();
    //reestablish bolt home position
    reverseBoltToSwitch();
    //turn motor current off
    digitalWrite(8, HIGH);
    //clear flag
    firstRun = 0;
  }
  
  //initial debounce on trigger from idle state. Safety measure.
  prevTrigState = currTrigState;
  currTrigState = (digitalRead(11) && !digitalRead(12));
  if(currTrigState && prevTrigState){
    //turn motor current on
    digitalWrite(8, LOW);
    //start flywheels
    OCR1B = 500; //go
    //wait for motor acceleration per delay schedule
    delay(feedDelayBase - delayReduction);
    //now that we will have just fired, apply recent shot comp
    delayReduction = recentShotCompensation;
    fire();
    //first sealed-in shot is over. Check trigger *quickly* for downness, fire again and again while down.
    while((PINB & 0b00001000) && !(PINB & 0b00010000)){
     fire(); 
    }
    if(!decelerateBoltToSwitch()) {reverseBoltToSwitch();}
    lastTriggerUp = millis();
    /*{
    OCR1B = 230; //shutdown
    delay(1699);
    if((millis() - lastTriggerUp)>boltShutdownTime){
      //turn bolt motor current off
      digitalWrite(8, HIGH);
    }
   
    delayReduction = recentShotCompensation;
   
    //turn motor current on
    digitalWrite(8, LOW);
    //start flywheels
    OCR1B = 500; //go
    //wait for motor acceleration per delay schedule
    delay(feedDelayBase - delayReduction);
    //now that we will have just fired, apply recent shot comp
    delayReduction = recentShotCompensation;
    fire();
    //first sealed-in shot is over. Check trigger *quickly* for downness, fire again and again while down.
    while((PINB & 0b00001000) && !(PINB & 0b00010000)){
     fire(); 
    }
    if(!decelerateBoltToSwitch()) {reverseBoltToSwitch();}
    lastTriggerUp = millis();
  } */
  } else {
    OCR1B = 230; //shutdown
    if((millis() - lastTriggerUp)>boltShutdownTime){
      //turn bolt motor current off
      digitalWrite(8, HIGH);
    }
    //feed delay management section
    if((millis() - lastTriggerUp)>recentShotTime){
      //"considerable" wheel speed is over, but rotor angle should not have been lost yet
      delayReduction = driveStartedCompensation;
    }
    
    if((millis() - lastTriggerUp)>driveCoastTime){
      //consider as 0 rpm cold start and reset to base feed delay
      delayReduction = 0;
    }
  }
  delay(5);
}
