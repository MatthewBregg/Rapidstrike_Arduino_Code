#include <Button_Debounce.h>

const int trigger_switch = 11;
const int other_trigger_switch = 10;
const int flywheel_mosfet = 8;
const int pusher_bjt = 9;

BasicDebounce trigger = BasicDebounce(trigger_switch, 8);

void setup() {
   pinMode(trigger_switch, INPUT_PULLUP);
   pinMode(other_trigger_switch, INPUT_PULLUP);
   pinMode(flywheel_mosfet,OUTPUT);
   trigger.AddSecondaryPin(other_trigger_switch);
}

void UpdateButtons() {
  trigger.update();
}

void loop() {
  UpdateButtons();

  if (trigger.query()) {
    digitalWrite(flywheel_mosfet,HIGH);
  } else {
    digitalWrite(flywheel_mosfet,LOW);
  }

}
