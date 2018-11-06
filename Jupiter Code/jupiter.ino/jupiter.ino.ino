#include <Button_Debounce.h>

const int trigger_switch = 11;
const int other_trigger_switch = 10;
const int cycle_switch = 6;
const int other_cycle_switch = 5;
const int flywheel_mosfet = 9;
const int pusher_bjt = 8;

BasicDebounce trigger = BasicDebounce(trigger_switch, 8);
BasicDebounce cycle = BasicDebounce(cycle_switch,8);

void setup() {
   pinMode(trigger_switch, INPUT_PULLUP);
   pinMode(other_trigger_switch, INPUT_PULLUP);
   pinMode(cycle_switch,INPUT_PULLUP);
   pinMode(other_cycle_switch,INPUT_PULLUP);
   pinMode(flywheel_mosfet,OUTPUT);
   pinMode(pusher_bjt,OUTPUT);
   trigger.AddSecondaryPin(other_trigger_switch);
   cycle.AddSecondaryPin(other_cycle_switch);
}

void UpdateButtons() {
  trigger.update();
  cycle.update();
}

// In MS, when did we last stop the flywheels?
long last_turned_down_flywheels = 0;

// Rev the flywheels, handle the FD, and then start the pusher.
void InitFiring() {
    const int FD_STAGE_1 = 1000;
    const int FD_STAGE_2 = 500;
    const long millis_since_rev = millis() - last_turned_down_flywheels;
    // Rev
    digitalWrite(flywheel_mosfet,HIGH); // Rev
    // Delay
    if (millis_since_rev < FD_STAGE_1 ) {
      // 75 too low on 7.6 V NiMH. 
      // 85 Is good!
      delay(85); // FD_STAGE_1 delay, revved within FD_STAGE_1 ms
    } else if (millis_since_rev < FD_STAGE_2) {
      delay(65); // FD_STAGE_2 delay, revved within 500 MS
    } else {
      // 6.6 V NIMH
      // 150 works! (97/98/99/100).
      // 112 marginally good? (95-100 AVE)
      // 102 marginally too low. (90-93 AVE)
      // 93 too Low
      // 7.6 V NIMH
      // 102 Marginally good ( 96 AVE)
      delay(105); // Full Feed Delay. Have not revved recently. 
    }
    // Fire and exit.
    digitalWrite(pusher_bjt,HIGH); // Start pushing
}

// Repeat cycling until we are done firing, then return. 
void RepeatCycle() {
   const int FIRE_TIMEOUT = 1000;
   bool prior_switch_reading = cycle.query(); 
   long started_at = millis();

    // Overall we want to
    // - Ensure a single ball is fired
    // - Then continue firing until the user releases the trigger. 
    // - If no balls are loaded/a jam occurs, allow the user to cancel firing.
    //   - For this, fall back to the trigger if 1 second goes by with no balls. 
    // - Lastly, attempt to end with the cycle switch open. However, again use the FIRE_TIMEOUT in case of jams. 
    while(true) {
      UpdateButtons();
      bool current_switch_reading = cycle.query();
      // If we have either gotten our cycle off successfully, or it's been a second with no
      // firing (assume empty/jam), fall back to trigger for ending cycle. 
      // TLDR; Wait for the cycle switch to fall, or if that doesn't occur in the FIRE_TIMEOUT
      // period, delegate to the user via the trigger.
      if ( !current_switch_reading && prior_switch_reading || millis() - started_at > FIRE_TIMEOUT) {
        // At this point, the user directly controls when the cycle ends via the trigger. 
        // This occurs when either a ball is fired, or we hit FIRE_TIMEOUT and assume JAM/OUTOFAMMO. 
        do {
          UpdateButtons();
        } while (trigger.query());
        
        const long time_started = millis();
        // Attempt to let the cycle switch go open, but don't do this for more than FIRE_TIMEOUT
        do {
          UpdateButtons();
        } while ( cycle.query() && (millis()-time_started) < FIRE_TIMEOUT );

        // User release trigger, cycle switch reset or was jammed, Fin.
        break;
      }
      
      prior_switch_reading = current_switch_reading;
    }
}

// Turn down the pusher and flywheels. 
void FinishFiring() {
   digitalWrite(pusher_bjt,LOW); // Stop pushing
   digitalWrite(flywheel_mosfet,LOW); // Stop the wheels
   last_turned_down_flywheels = millis();
}

void FireBlaster() {
    InitFiring();
    RepeatCycle();
    FinishFiring();
    // For testing FD
    delay(995);
    InitFiring();
    RepeatCycle();
    FinishFiring();
    // For testing FD
   
    
}

void loop() {
  UpdateButtons();
  if (trigger.query()) {
    FireBlaster();
  }
  
}
