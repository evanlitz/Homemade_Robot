// Limit-sensor probe for the AR4 MK5 on a Teensy 4.1. MOVES NOTHING.
//
// Reads the six limit pins (26-31, J1..J6) twice a second and prints each
// one twice: as a plain INPUT (how the ar4_ros_driver fork reads J4-J6) and
// with INPUT_PULLUP (how it reads J1-J3). It never touches the step or
// direction pins, so no motor can be driven.
//
// Use it to find out what each joint's sensor actually does:
//   1. Flash this, open the Serial Monitor at 115200.
//   2. With the arm at rest, note each joint's idle reading.
//   3. Trigger one sensor at a time -- press the switch, or hold the joint's
//      magnet/flag at its sensor -- and note which column changes, and to what.
//
// The fork's MK5 table (LIMIT_SENSOR_PRESSED_STATE_MK5 and
// applyLimitSwitchInputModeForModel in AR4_teensy.ino) has to match:
//   pin reads HIGH when triggered, plain INPUT    -> pressed state HIGH, INPUT
//   pin reads LOW  when triggered, needs pull-up  -> pressed state LOW,  INPUT_PULLUP
// A joint whose reading never changes is a wiring or sensor fault, not a
// firmware setting.
//
// Afterwards, flash AR4_teensy.ino again before using the arm.

const int LIMIT_PINS[6] = {26, 27, 28, 29, 30, 31};

int readAs(int pin, int mode) {
  pinMode(pin, mode);
  delayMicroseconds(200);  // let the pull-up settle before sampling
  return digitalRead(pin);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("limit probe: nothing will move. columns: plain INPUT / INPUT_PULLUP");
}

void loop() {
  for (int i = 0; i < 6; i++) {
    int plain = readAs(LIMIT_PINS[i], INPUT);
    int pulled = readAs(LIMIT_PINS[i], INPUT_PULLUP);
    Serial.printf("J%d %s/%s   ", i + 1, plain ? "H" : "L", pulled ? "H" : "L");
  }
  Serial.println();
  delay(500);
}
