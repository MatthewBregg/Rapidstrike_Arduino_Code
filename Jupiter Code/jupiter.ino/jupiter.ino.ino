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

void FireBlaster() {
    // Rev
    digitalWrite(flywheel_mosfet,HIGH); // Rev
    delay(180); // Feed Delay
    digitalWrite(pusher_bjt,HIGH); // Start pushing
    
    bool prior_switch_reading = cycle.query(); 
    long started_at = millis();

    // Wait for the switch to encounter a falling edge
    while(true) {
      UpdateButtons();
      bool current_switch_reading = cycle.query();
      // If we have either gotten our cycle off successfully, or it's been a second with no
      // firing (assume empty/jam), fall back to trigger for ending cycle. 
      if ( !current_switch_reading && prior_switch_reading || millis() - started_at > 1000) {
        do {
          UpdateButtons();
        } while (trigger.query());
        // We have fired at least one ball (or detected a jam, and trigger is no longer held down. 
        // End firing cycle.
        break;
      }
      prior_switch_reading = current_switch_reading;
    }
    digitalWrite(pusher_bjt,LOW); // Stop pushing
    delay(100); // In case of follow ups
    digitalWrite(flywheel_mosfet,LOW); // Stop the wheels
}

void loop() {
  UpdateButtons();
  if (trigger.query()) {
    FireBlaster();
  }
  
}
