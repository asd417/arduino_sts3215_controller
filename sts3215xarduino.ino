#include "Arduino.h"
#include <sts3215.h>

#include <LiquidCrystal_I2C.h>
#include <stdint.h>

const uint8_t MODEBUTTON_PIN = 3;
// button to GND, internal pull-up: pressed = LOW
const uint8_t ACTIONBUTTON_PIN = 4;
const uint8_t LED_PIN = LED_BUILTIN; // on-board "L" LED (pin 13): HIGH = on

const uint8_t MAXCHAR = 16;  // LCD columns
const uint8_t LINEBUF = 41;  // up to 40 chars per line; longer than MAXCHAR scrolls
char screenLine1[LINEBUF];
char screenLine2[LINEBUF];

const uint16_t SCROLL_MS = 400; // time per scroll step
const uint8_t SCROLL_GAP = 3;   // blank columns between the end and the restart
uint16_t scrollPos = 0; // ponytail: wraps after ~7 h, one visible jump then
unsigned long lastScroll = 0;

uint8_t mode = 0;
// mode 0: ping
// mode 1: set target ID with the pot
// mode 2: connect
// mode 3: motor ctl (pot drives the position between minPos and maxPos)
// mode 4: motor calib (torque off, pose by hand: set min -> set max -> done)
const uint8_t MODE_COUNT = 5;
bool modeEntered = true; // set on a mode change; modes that need setup clear it
const uint8_t screenTimerMax = 200;
uint8_t screenTimer = 0;
unsigned long lastScreenTime = 0;
// the screen only updates when the screenTimer hits screenTimerMax
bool screenUpdated = 0;

const uint16_t buttonRefireMax = 600;
unsigned long lastButtonTime = 0;
uint16_t buttonRefireTimer = 0;


uint8_t targetID = 1;
HardwareSerial *m = nullptr;
LiquidCrystal_I2C lcd(0x27, 16, 2);

int readButton(uint8_t pin) {
  int p = digitalRead(pin);
  if(!p && millis() - lastButtonTime >= 500 )
  {
    lastButtonTime = millis();
    return 0;
  }
  return 1;
}

void queueLine(char *buffer, const char *linefmt, ...) {
  va_list ap;
  va_start(ap, linefmt);
  vsnprintf(buffer, LINEBUF, linefmt, ap);
  va_end(ap);
}

void queueScreen(const char *line1, const char *line2) {
  memcpy(&screenLine1, (void *)line1, MAXCHAR - 1);
  memcpy(&screenLine2, (void *)line2, MAXCHAR - 1);
  screenLine1[MAXCHAR - 1] = '\0';
  screenLine2[MAXCHAR - 1] = '\0';
}

void cycleMode() { mode = (++mode) % MODE_COUNT; }

// Writes all MAXCHAR columns of a row (padded with spaces, so no lcd.clear()).
// Lines longer than MAXCHAR show a window starting at scrollPos.
void printLine(uint8_t row, const char *s) {
  uint8_t len = strlen(s);
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < MAXCHAR; i++) {
    uint8_t j = len > MAXCHAR ? (scrollPos + i) % (len + SCROLL_GAP) : i;
    lcd.write(j < len ? s[j] : ' ');
  }
}
void updateScreenNow()
{
  printLine(0, screenLine1);
  printLine(1, screenLine2);
}
void setup() {
  pinMode(MODEBUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTIONBUTTON_PIN, INPUT_PULLUP);

  lcd.init();      // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  queueLine(screenLine1, "Welcome");
  updateScreenNow();
  delay(1000);
}


void updateScreen() {
  if (millis() - lastScreenTime >= 500 &&
      screenUpdated) { // update when 0 and when screen needs updating
    updateScreenNow();
    lastScreenTime = millis();
    screenUpdated = 0;
  }
  screenTimer = (++screenTimer) % screenTimerMax;

  if (millis() - lastScroll >= SCROLL_MS) {
    lastScroll = millis();
    scrollPos++;
    if (strlen(screenLine1) > MAXCHAR || strlen(screenLine2) > MAXCHAR)
      updateScreenNow();
  }
}

const uint16_t RESULT_MS = 2000; // how long a result stays before the idle text returns

// Puts idle text on line 2 once RESULT_MS has passed since the last button
// press (reuses lastButtonTime, so no extra timer). An empty line 2 (just
// after a mode change) gets it right away.
void idleLine2(const char *text) {
  if ((!screenLine2[0] || millis() - lastButtonTime >= RESULT_MS) &&
      strcmp(screenLine2, text)) {
    queueLine(screenLine2, "%s", text);
    screenUpdated = 1;
  }
}

void pingMode() {
  queueLine(screenLine1, "Mode: Ping");
  if (!readButton(ACTIONBUTTON_PIN)) {
    screenUpdated = 1; // flag update
    int servoerror = 0;
    int error = ping(m, targetID, &servoerror);
    if (error) {
      queueLine(screenLine2, "Error %d", error);
    } else {
      queueLine(screenLine2, "Ping Success!");
    }
  } else {
    idleLine2("...");
  }
}

// Shows the current targetID and the pot's pick; only an action press applies
// it, so turning the pot for motor ctl doesn't change the ID.
void idEncoderMode() {
  queueLine(screenLine1, "Set ID: now %d", targetID);
  uint8_t pick = map(analogRead(A0), 0, 1023, 1, 16);
  if (!readButton(ACTIONBUTTON_PIN)) {
    targetID = pick;
    queueLine(screenLine2, "ID set to %d", pick);
    screenUpdated = 1;
    return;
  }
  char buf[MAXCHAR + 1];
  snprintf(buf, sizeof(buf), "Pot %d: press", pick);
  idleLine2(buf);
}

void connectMode() {
  queueLine(screenLine1, "Mode: Connect");
  if(m)
  {
    idleLine2("Servo Connected");
  }
  else
  {
    idleLine2("Connect and Press");
  }
  if (!readButton(ACTIONBUTTON_PIN)) {
    screenUpdated = 1; // flag update
    int r = openPort(1000000, &m, nullptr);

    int servoerror = 0;
    int error = ping(m, targetID, &servoerror);
    if(!error && !servoerror)
    {
      queueLine(screenLine2, "Servo Connected");
    }
    else {
      if(servoerror) queueLine(screenLine2, "SE: %d", error);
      else queueLine(screenLine2, "E: %d", error);
      m = nullptr;
    }
  }
}

int16_t minPos = 0;    // set in calib mode (RAM only, lost on reset)
int16_t maxPos = 4095;
const uint8_t CTL_DEADBAND = 8;  // pot jitter smaller than this isn't sent
const uint8_t CTL_SEND_MS = 50;  // at most one goal write per this

// Pot sets the goal between minPos and maxPos. Action starts/stops driving, so
// the pot can be lined up before the servo follows it.
void motorCtlMode() {
  static bool driving = false;
  static int16_t lastGoal = 0;
  static unsigned long lastSend = 0;
  if (modeEntered) {
    modeEntered = false;
    driving = false;
  }
  queueLine(screenLine1, driving ? "Motor Ctl: on" : "Motor Ctl: off");

  long g = map(analogRead(A0), 0, 1023, minPos, maxPos); // min > max just reverses the pot
  int16_t goal = constrain(g, 0, 4095);

  if (!readButton(ACTIONBUTTON_PIN)) {
    screenUpdated = 1;
    if (!m) {
      queueLine(screenLine2, "Connect first");
      return;
    }
    int servoerror = 0;
    int error = 0;
    if (!driving) {
      // goal first, so torque-on doesn't jump to a stale goal register
      error = setGoalPosition(m, targetID, goal, &servoerror);
      if (!error)
        error = setTorqueEnable(m, targetID, 1, &servoerror);
      if (!error) {
        driving = true;
        lastGoal = goal;
      }
    } else {
      driving = false;
      error = setTorqueEnable(m, targetID, 0, &servoerror);
    }
    if (error)
      queueLine(screenLine2, "E: %d", error);
    return;
  }

  if (driving && abs(goal - lastGoal) >= CTL_DEADBAND &&
      millis() - lastSend >= CTL_SEND_MS) {
    lastSend = millis();
    int servoerror = 0;
    int error = setGoalPosition(m, targetID, goal, &servoerror);
    if (error) {
      driving = false; // stop sending; don't retry every loop
      queueLine(screenLine2, "E: %d", error);
      screenUpdated = 1;
      return;
    }
    lastGoal = goal;
  }

  char buf[MAXCHAR + 1];
  snprintf(buf, sizeof(buf), "Goal %d", goal);
  idleLine2(buf);
}

// Torque goes off on entry so the horn turns by hand. Each action press stores
// the present position: set min -> set max -> done (a press at done restarts).
void motorCalibMode() {
  static uint8_t step = 0; // 0: set min, 1: set max, 2: done
  queueLine(screenLine1, "Mode: Calib");
  if (modeEntered) {
    modeEntered = false;
    step = 0;
    if (m) {
      int servoerror = 0;
      int error = setTorqueEnable(m, targetID, 0, &servoerror);
      if (error)
        queueLine(screenLine2, "E: %d", error); // idleLine2 leaves it up for RESULT_MS
    }
  }
  if (!m)
    idleLine2("Connect first");
  else
    idleLine2(step == 0 ? "Set min" : step == 1 ? "Set max" : "Done");

  if (!readButton(ACTIONBUTTON_PIN) && m) {
    screenUpdated = 1;
    if (step == 2) { // restart
      step = 0;
      screenLine2[0] = '\0'; // show "Set min" now, not after RESULT_MS
      return;
    }
    int servoerror = 0;
    int16_t pos = 0;
    int error = getPresentPosition(m, targetID, &pos, &servoerror);
    if (error) {
      queueLine(screenLine2, "E: %d", error); // step unchanged: press again
      return;
    }
    if (step == 0) {
      minPos = pos;
      queueLine(screenLine2, "Min %d", pos);
    } else {
      maxPos = pos;
      queueLine(screenLine2, "Max %d", pos);
    }
    step++;
  }
}
void loop() {

  if (!readButton(MODEBUTTON_PIN)) {
    cycleMode();
    modeEntered = true;
    screenLine2[0] = '\0'; // let the new mode's idle text show without the RESULT_MS wait
    screenUpdated = 1; // flag update
  }
  switch (mode) {
  case 0:
    pingMode();
    break;
  case 1:
    idEncoderMode();
    break;
  case 2:
    connectMode();
    break;
  case 3:
    motorCtlMode();
    break;
  case 4:
    motorCalibMode();
    break;
  default:
    break;
  }
  
  // digitalWrite(LED_PIN, digitalRead(BUTTON_PIN) == LOW ? HIGH : LOW);
  // digitalWrite(LED_PIN, HIGH); delay(500);
  // digitalWrite(LED_PIN, LOW);  delay(500);
  updateScreen();
}
