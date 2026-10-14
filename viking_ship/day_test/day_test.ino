// Viking Ship Clocktower: ALL-DAY STABILITY TEST (day_test.ino, built from row_test.ino v1.1 + bells)
// Runs the full show on the hour from 9:00 to 20:00 by the DS3231 clock, rowing the hour (9, 10, 11,
// 12, 1 ... 8), and keeps score: shows, faults, power-up count, extend/retract times and a HOME drift
// check after every show. Read the LCD at a glance, or the Serial Monitor log line after each show.
//
// SCHEDULE: shows at START_HOUR:00 through END_HOUR:00 (9:00 to 20:00 = 12 shows). Outside those hours
//   the ship stays parked and the LCD says when the next show is. f = fast mode (a show every 5 minutes,
//   any hour) to check the schedule without waiting; f again to go back to hourly.
// DRIFT CHECK: after each retract the carriage creeps back onto HOME and reports how many steps off
//   zero it trips (a few steps = normal switch repeatability; more means lost steps), then re-zeros and
//   parks. More than 1" off is a fault.
// FAULTS: any fault stops the motor and is counted and logged with the time. The next scheduled show
//   recalibrates first and carries on, so one bad show doesn't end the whole day's test.
// POWER-UPS: counted in EEPROM, so if the Uno resets on its own during the day (brownout, noise), the
//   boot count on the LCD goes up. z clears it before a new day.
// Set the clock first with bringup_rtc_lcd.ino (serial: T YYYY MM DD HH MM SS).
//
// SEQUENCE
//   1. POWER-UP CALIBRATION (automatic, every time the Uno starts or resets, e.g. after a power loss):
//      creeps to the HOME switch, backs off, touches it again slowly and calls that 0. Then creeps out
//      to the FAR switch, measures the real travel, and parks just off HOME. Same routine that passed
//      in travel_test.ino on 2026-10-09. k re-runs it any time. Set AUTO_CALIBRATE false to disable.
//   2. Show (g): 4 bells ring out fully (/mp3/0005.mp3, BELL_MS), then the cart sound starts, extend to 1" short of FAR timed so the clip's thud lands as the
//      ship stops, hold, row one stroke per hour count (row sound at each oar bite), hold, cart sound
//      again, retract timed the same way so the thud lands as it parks just off HOME. Motor released.
//
// CART SOUND SYNC: /mp3/0002.mp3 ends in a thud at exactly 7.5 s (CART_THUD_MS).
//   The sketch sends the play command, waits for the DFPlayer to actually start (EXTEND_DF_LAG_MS /
//   RETRACT_DF_LAG_MS, separate because the first sound after a quiet spell starts much later), then
//   starts a move whose cruise speed is computed from the calibrated travel so the move takes 7.5 s.
//   After each move it prints the measured time and nudges that direction's trim so the next move
//   lands closer. Tune by ear: thud BEFORE the stop = raise that direction's lag, AFTER = lower it.
//   2026-10-09 bench run: extend thud ~1/4 s late, retract close. Lags and starting trims set from it.
//   No door-zone slowdown on these moves: the door servos open the doors first.
//
// ROWING TUNE (v1.1, 2026-10-09): faster surge, retreat that fights the boat's momentum.
//   Surge: scull curve (jumps off the catch), 1200 ms, peak ~8.5 in/s (~2,890 steps/s).
//   Retreat: speed ~ u^2 * (1-u)^0.5. Starts barely moving (forward momentum resisting the fall-back),
//   keeps speeding up, is fastest ~80% through, then the next catch stops it hard. 1400 ms, peak
//   ~9.4 in/s (~3,180 steps/s). Stroke 7", catch 100 ms, no pause after the surge, 400 ms between
//   counts, ~2.7 s per stroke.
// ROW SOUND (drum + "Row!"): the "Row!" voice should land where it did before (at the catch). The clip
//   now starts with a drum, so the sketch starts it ROW_VOICE_OFFSET_MS early, during the end of the
//   retreat. That command is sent one byte at a time (DF_BYTE_GAP_US apart) so the moving motor never
//   loses more than ~1 ms of steps. Set ROW_VOICE_OFFSET_MS = where "Row" starts in the new clip
//   minus where it started in the old clip (Audacity). 0 = old behavior (sent at the catch).
// TRAVEL: accel 12 in/s^2, hold 800 ms before rowing and 600 ms after, extend stops 1" short of FAR.
//
// LIMIT SWITCHES (wired normally closed, so a cut or unplugged wire reads as "hit" and stops motion)
//   HOME switch (motor end): COM -> GND, NC -> D8
//   FAR switch (idler end):  COM -> GND, NC -> D9
//
// DFPLAYER MINI: VCC/GND from its own 5V supply, its GND also tied to an Uno GND (one wire),
//   D7 -> 1 kOhm -> RX (TX unused), SPK_1/SPK_2 -> speaker.
//   microSD (FAT32): /mp3/0001.mp3 = drum + "Row!", /mp3/0002.mp3 = cart sound (thud at 7.5 s).
//
// STEPPER: TMC2209, write-only UART on A0, all DIPs off, STEP D2, DIR D3, EN D4.
//
// Libraries: TMCStepper (teemuatlut), AccelStepper, DFRobotDFPlayerMini, RTClib, LiquidCrystal I2C
//
// Serial Monitor (9600 baud):
//   s        status: time, shows, faults, power-ups, drift, next show
//   f        fast mode on/off (show every 5 minutes)
//   z        clear the power-up counter
//   w        show the limit switch states
//   g        run a show now (calibrates first if needed; hour count = current hour)
//   k        calibrate again
//   1 .. 9   hour count (0 = 12). Default 3.
//   x        finish: end rowing after this stroke, then retract and park
//   p        stop now (motor released, sound stopped, needs k or g to recalibrate)
//   h        park (return to just off HOME)
//   t        test the row sound    c  test the cart sound    b  test the bells    n  sound on/off    + / -  volume
//   m        quiet mode (StealthChop) / strong mode (SpreadCycle)
//   r        flip motor direction (if calibration heads to the wrong switch)

#include <Wire.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <DFRobotDFPlayerMini.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ---- Day test schedule and score ----
const uint8_t START_HOUR = 9;      // first show 9:00
const uint8_t END_HOUR   = 20;     // last show 20:00 (8 pm)
bool fastMode = false;             // f: a show every 5 minutes, any hour
uint16_t showCount = 0, faultCount = 0, bootCount = 0;
long lastShowKey = -1;
long lastDrift = 0, maxDrift = 0;
long lastExtendMs = 0, lastRetractMs = 0;
uint8_t lastFaultH = 0, lastFaultM = 0;

// ---- Pins ----
const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;
const uint8_t PIN_DF_RX = 6;      // from DFPlayer TX (not needed)
const uint8_t PIN_DF_TX = 7;      // to DFPlayer RX
const uint8_t PIN_HOME = 8;       // HOME limit switch (motor end)
const uint8_t PIN_FAR  = 9;       // FAR limit switch (idler end)
const uint8_t PIN_UART_TX = A0;   // to TMC2209 PDN_UART
const uint8_t PIN_UART_RX = A1;   // not connected yet
const uint8_t PIN_DF_BUSY = A3;   // optional: DFPlayer BUSY (module pin 16), LOW while playing

// ---- Switches and startup ----
const bool SWITCH_NC      = true;   // true: COM-GND, NC-pin (reads HIGH when hit or wire cut)
bool       dirInvert      = true;   // 2026-10-09: DIR was backwards on the bench
const bool AUTO_CALIBRATE = true;   // calibrate on every power-up / reset
const uint16_t STARTUP_DELAY_MS = 2000;   // let the 12V supply and driver settle first

// ---- Distances and speeds, in steps (1/4 step: 338.67 steps per inch) ----
const long  HOME_BACKOFF     = 170;    // 0.5": back off HOME before the slow second touch
const long  PARK_OFFSET      = 85;     // 0.25": park this far out from the HOME trip point
const long  FAR_MARGIN       = 339;    // 1": extend stops this far short of the FAR trip point
const long  MAX_SEEK_STEPS   = 13547;  // 40": give up if a switch never trips (bench rail is ~34")
const long  DOOR_ZONE_START  = 47;     // 0.14": bow meets the doors (only used by h park and cal park now)
const long  DOOR_ZONE_END    = 4064;   // 12": doors fully open
const float RETRACT_SPEED    = 1355;   // 4 in/s, used by h park and the calibration park
const float DOOR_ZONE_SPEED  = 508;    // 1.5 in/s
const float TRAVEL_ACCEL     = 4064;   // 12 in/s^2
const float TIMED_MAX_SPEED  = 3400;   // ~10 in/s: proven clean in travel_test, cap for timed moves
const float SEEK_SPEED       = 339;    // 1 in/s, calibration
const float SLOW_SEEK_SPEED  = 85;     // 0.25 in/s, second touch on HOME
const uint16_t HOLD_BEFORE_ROW_MS = 800;
const uint16_t HOLD_AFTER_ROW_MS  = 600;

// ---- Rowing tune (paste new values from the Longship Motion Lab's "Values for the sketch") ----
const long     ROW_STROKE        = 2371;   // 7"
const uint16_t ROW_DRIVE_MS      = 1200;   // outward surge, peak ~8.5 in/s (scull curve). Was 1627.
const uint16_t ROW_RECOVER_MS    = 1400;   // inward retreat, peak ~9.4 in/s near the end (momentum curve). Was 1150.
const byte     STROKES_PER_COUNT = 1;
const uint16_t CATCH_PAUSE_MS    = 100;    // oar bite delay: row sound fires here, then the surge
const uint16_t FINISH_PAUSE_MS   = 0;      // no pause after the surge: the water resists immediately
const uint16_t COUNT_PAUSE_MS    = 400;
const uint16_t DRIVE_CURVE[33] = {
  0, 9, 27, 52, 81, 115, 151, 191, 232, 275, 320,
  365, 410, 456, 501, 546, 590, 633, 675, 715, 753, 790,
  824, 855, 884, 910, 934, 953, 970, 983, 992, 998, 1000
};
const uint16_t RECOVER_CURVE[33] = {   // speed ~ u^2 * (1-u)^0.5: slow start, fastest ~80% in, hard stop
  0, 0, 1, 2, 4, 8, 13, 21, 31, 43, 58,
  76, 98, 122, 150, 181, 216, 254, 295, 339, 387, 437,
  490, 546, 603, 661, 719, 778, 834, 888, 937, 977, 1000
};

// ---- Driver and sound ----
const int      MICROSTEP      = 4;
const uint16_t RUN_CURRENT_MA = 1000;
const float    HOLD_FRACTION  = 0.5;
bool           quietMode      = true;
const float    R_SENSE        = 0.11;
const uint8_t  ROW_TRACK      = 1;     // /mp3/0001.mp3
const uint8_t  CART_TRACK     = 2;     // /mp3/0002.mp3
const uint8_t  BELL_TRACK     = 5;     // /mp3/0005.mp3 = 4 bells before the ship comes out
const bool     BELL_BEFORE_SHOW = true;
const uint16_t BELL_MS        = 6000;  // FULL length of 0005.mp3 incl. the ringing tail (Audacity): SET THIS
const uint16_t BELL_GAP_MS    = 300;   // quiet beat after the last ring before the cart sound starts
const bool     USE_BUSY_PIN   = false; // true once BUSY is wired to A3: waits for the bell to really end
const uint16_t EXTEND_DF_LAG_AFTER_BELL_MS = 180;  // DFPlayer is awake after the bell, so the extend lag is short
const bool     CART_SOUND     = true;  // cart clip on extend (and retract, see below)
const bool     CART_ON_RETRACT = true; // play it again on the retract so the thud lands at park
const uint16_t CART_THUD_MS   = 7500;  // thud position in 0002.mp3 (measured in Audacity)
const uint16_t EXTEND_DF_LAG_MS  = 500;  // wait after the cart play command before the extend starts
const uint16_t RETRACT_DF_LAG_MS = 180;  // same for the retract (DFPlayer is awake by then, so shorter)
const long     EXTEND_TRIM_START  = 160; // starting trims from the 2026-10-09 run (moves ran 160 / 80 ms long)
const long     RETRACT_TRIM_START = 80;
const uint16_t ROW_VOICE_OFFSET_MS = 420; // drum to "Row!" in the new 0001.mp3 (2026-10-09). Voice late: raise. Early: lower.
const uint16_t DF_BYTE_GAP_US  = 2000;   // gap between bytes when sending a command while the motor moves
int            volumeLevel    = 18;    // 0..30
bool           soundOn        = true;

const float FOLLOW_GAIN   = 20.0;
const float MAX_STEP_RATE = 3800;
const float MIN_SPEED     = 60;

SoftwareSerial tmcSerial(PIN_UART_RX, PIN_UART_TX);
SoftwareSerial dfSerial(PIN_DF_RX, PIN_DF_TX);
TMC2209Stepper driver(&tmcSerial, R_SENSE, 0);
DFRobotDFPlayerMini player;
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
RTC_DS3231 rtc;
LiquidCrystal_I2C* lcd = nullptr;
bool rtcOk = false;

enum State {
  IDLE,
  CAL_SEEK_HOME, CAL_BACKOFF, CAL_SLOW_HOME, CAL_SEEK_FAR, CAL_PARK,
  DRIFT_SEEK, DRIFT_PARK,
  BELL, EXTEND_CUE, EXTEND, HOLD_IN, RECOVER, CATCH, DRIVE, FINISH, REST, HOLD_OUT,
  RETRACT_CUE, RETRACT, PARKING
};
State state = IDLE;
bool calibrated = false;
bool showAfterCal = false;
long farTrip = 0, extendPos = 0;
int  hourCount = 3, countNow = 0;
byte strokeInCount = 0;
bool finishRequested = false;
unsigned long pauseStart = 0;
long markPos = 0, entryPos = 0;

// ---- Timed moves (cart thud sync) ----
long extendTrimMs = EXTEND_TRIM_START;    // learned corrections: positive = moves have been running long
long retractTrimMs = RETRACT_TRIM_START;
bool timedIsExtend = true;
// ---- Row sound sent in pieces while the motor runs ----
byte dfPkt[10]; byte dfIdx = 10; unsigned long dfLastByte = 0;
bool rowCueSent = false;
unsigned long timedStart = 0;
uint16_t timedTarget = 0;

// ---- Limit switches (sampled once per ms, 5 matching reads = 5 ms debounce) ----
byte homeCnt = 0, farCnt = 0;
bool homeHit = false, farHit = false;
unsigned long lastSample = 0;
bool rawHit(uint8_t pin) { return SWITCH_NC ? digitalRead(pin) == HIGH : digitalRead(pin) == LOW; }
void updateSwitches() {
  unsigned long now = millis();
  if (now == lastSample) return;
  lastSample = now;
  if (rawHit(PIN_HOME)) { if (homeCnt < 5) homeCnt++; } else homeCnt = 0;
  if (rawHit(PIN_FAR))  { if (farCnt  < 5) farCnt++;  } else farCnt  = 0;
  homeHit = homeCnt >= 5;
  farHit  = farCnt  >= 5;
}

// ---- Trapezoid travel with optional door-zone slowdown ----
long mvTarget = 0; float mvVmax = 0, mvSpeed = 0; bool mvDoor = false; unsigned long mvLast = 0;
void startTravel(long target, float vmax, bool door) {
  mvTarget = target; mvVmax = vmax; mvDoor = door; mvSpeed = MIN_SPEED; mvLast = micros();
  stepper.setSpeed(target >= stepper.currentPosition() ? MIN_SPEED : -MIN_SPEED);
}
bool runTravel() {
  long pos = stepper.currentPosition();
  long rem = mvTarget - pos;
  if (rem == 0) { stepper.setSpeed(0); return true; }
  int dir = rem > 0 ? 1 : -1;
  unsigned long now = micros();
  if (now - mvLast >= 2000) {
    float dt = (now - mvLast) / 1e6; mvLast = now;
    float cap = mvVmax;
    if (mvDoor) {
      if (pos >= DOOR_ZONE_START && pos <= DOOR_ZONE_END) cap = min(cap, DOOR_ZONE_SPEED);
      else if (dir > 0 && pos < DOOR_ZONE_START && mvTarget > DOOR_ZONE_START)
        cap = min(cap, sqrt(DOOR_ZONE_SPEED * DOOR_ZONE_SPEED + 2 * TRAVEL_ACCEL * (DOOR_ZONE_START - pos)));
      else if (dir < 0 && pos > DOOR_ZONE_END && mvTarget < DOOR_ZONE_END)
        cap = min(cap, sqrt(DOOR_ZONE_SPEED * DOOR_ZONE_SPEED + 2 * TRAVEL_ACCEL * (pos - DOOR_ZONE_END)));
    }
    cap = min(cap, sqrt(2 * TRAVEL_ACCEL * labs(rem)));
    if (mvSpeed < cap) mvSpeed = min(cap, mvSpeed + TRAVEL_ACCEL * dt);
    else               mvSpeed = max(cap, mvSpeed - TRAVEL_ACCEL * dt);
    if (mvSpeed < MIN_SPEED) mvSpeed = MIN_SPEED;
    stepper.setSpeed(dir * mvSpeed);
  }
  stepper.runSpeed();
  return false;
}

// Cruise speed for a trapezoid (accel = decel = TRAVEL_ACCEL) that covers dist steps in ms.
// T = D/v + v/a  ->  v = (aT - sqrt(a^2 T^2 - 4aD)) / 2
float speedForTime(long dist, long ms) {
  float a = TRAVEL_ACCEL, T = ms / 1000.0;
  float disc = a * a * T * T - 4.0 * a * dist;
  if (disc <= 0) return TIMED_MAX_SPEED;            // can't make it even flat out: go as fast as allowed
  float v = (a * T - sqrt(disc)) / 2.0;
  return constrain(v, MIN_SPEED, TIMED_MAX_SPEED);
}
void startTimedTravel(long target, bool isExtend) {
  timedIsExtend = isExtend;
  long dist = labs(target - stepper.currentPosition());
  long planMs = (long)CART_THUD_MS - (isExtend ? extendTrimMs : retractTrimMs);
  float v = speedForTime(dist, planMs);
  if (v >= TIMED_MAX_SPEED) Serial.println(F("Note: move can't finish in 7.5 s at the speed cap; thud will land early."));
  timedTarget = CART_THUD_MS;
  timedStart = millis();
  startTravel(target, v, false);      // no door-zone slowdown: the servos open the doors first
}
void reportTimed(const __FlashStringHelper* name) {   // called while stopped
  long took = millis() - timedStart;
  long err = took - (long)timedTarget;
  if (timedIsExtend) lastExtendMs = took; else lastRetractMs = took;
  Serial.print(name); Serial.print(F(" took ")); Serial.print(took / 1000.0, 2);
  Serial.print(F(" s (target 7.50). Thud vs stop: "));
  if (err > 0) { Serial.print(F("thud ")); Serial.print(err); Serial.println(F(" ms early")); }
  else if (err < 0) { Serial.print(F("thud ")); Serial.print(-err); Serial.println(F(" ms late")); }
  else Serial.println(F("on the mark"));
  long &trim = timedIsExtend ? extendTrimMs : retractTrimMs;
  trim = constrain(trim + err / 2, -1000L, 1000L);   // half-step correction for next time
}

// ---- Shaped rowing moves ----
long segFrom = 0, segDist = 0; uint16_t segMs = 1; const uint16_t* segCurve = DRIVE_CURVE;
unsigned long segStart = 0, lastSpeedCalc = 0;
void startSegment(long dist, uint16_t ms, const uint16_t* curve) {
  segFrom = stepper.currentPosition(); segDist = dist; segMs = ms; segCurve = curve;
  segStart = millis(); lastSpeedCalc = 0;
}
bool runSegment() {
  unsigned long now = millis(), el = now - segStart;
  long target = segFrom + segDist;
  if (el >= segMs) {
    long err = target - stepper.currentPosition();
    if (err == 0) { stepper.setSpeed(0); return true; }
    float sp = err * FOLLOW_GAIN;
    if (sp > 0 && sp < 150) sp = 150;
    if (sp < 0 && sp > -150) sp = -150;
    stepper.setSpeed(sp); stepper.runSpeed();
    return false;
  }
  if (now - lastSpeedCalc >= 2) {
    lastSpeedCalc = now;
    float f = el * 32.0 / segMs;
    int i = (int)f; if (i > 31) i = 31;
    float w = f - i;
    float p = segCurve[i] + (segCurve[i + 1] - (float)segCurve[i]) * w;
    long want = segFrom + (long)(segDist * p / 1000.0);
    float ff = segDist * ((float)segCurve[i + 1] - segCurve[i]) / 1000.0 * 32000.0 / segMs;
    stepper.setSpeed(constrain(ff + FOLLOW_GAIN * (want - stepper.currentPosition()), -MAX_STEP_RATE, MAX_STEP_RATE));
  }
  stepper.runSpeed();
  return false;
}

// ---- Driver, sound, LCD ----
void configureDriver() {
  driver.begin();
  driver.I_scale_analog(false);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(RUN_CURRENT_MA, HOLD_FRACTION);
  driver.microsteps(MICROSTEP);
  driver.intpol(true);
  driver.en_spreadCycle(!quietMode);
  driver.pwm_autoscale(true);
}
void play(uint8_t track) { if (soundOn) player.playMp3Folder(track); }   // only call while stopped

// Same "play /mp3/NNNN.mp3" command (0x12) the library sends, but queued and sent one byte per
// DF_BYTE_GAP_US by serviceDf(), so each byte blocks the loop for only ~1 ms while the motor runs.
void queuePlay(uint8_t track) {
  if (!soundOn) return;
  uint16_t sum = 0xFF + 0x06 + 0x12 + 0x00 + 0x00 + track;
  uint16_t chk = (uint16_t)(0 - sum);
  byte p[10] = {0x7E, 0xFF, 0x06, 0x12, 0x00, 0x00, track, (byte)(chk >> 8), (byte)(chk & 0xFF), 0xEF};
  memcpy(dfPkt, p, 10);
  dfIdx = 0;
  dfLastByte = micros() - DF_BYTE_GAP_US;
}
void serviceDf() {
  if (dfIdx >= 10) return;
  if (micros() - dfLastByte < DF_BYTE_GAP_US) return;
  dfSerial.write(dfPkt[dfIdx++]);
  dfLastByte = micros();
}

uint8_t findLcd() {
  const uint8_t cands[] = {0x27, 0x3F, 0x20, 0x38};
  for (uint8_t a : cands) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) return a; }
  return 0;
}
void printTwo(Print &p, int v) { if (v < 10) p.print('0'); p.print(v); }
bool inWindow(uint8_t h) { return fastMode || (h >= START_HOUR && h <= END_HOUR); }
void printNext(Print &p, DateTime n) {   // "10:00", "9:00 tmrw" style
  uint8_t h = n.hour(), m = n.minute();
  if (fastMode) { uint8_t nm = (m / 5 + 1) * 5; if (nm >= 60) { nm = 0; h = (h + 1) % 24; } printTwo(p, h); p.print(':'); printTwo(p, nm); return; }
  if (h < START_HOUR) { printTwo(p, START_HOUR); p.print(F(":00")); return; }
  if (h >= END_HOUR) { printTwo(p, START_HOUR); p.print(F(":00 tmrw")); return; }
  printTwo(p, h + 1); p.print(F(":00"));
}
void printClock(Print &p, DateTime n) { printTwo(p, n.hour()); p.print(':'); printTwo(p, n.minute()); p.print(':'); printTwo(p, n.second()); }
void updateLcd() {   // only called while stopped, so it never stalls a move
  if (!lcd) return;
  DateTime n = rtcOk ? rtc.now() : DateTime();
  lcd->setCursor(0, 0); lcd->print(fastMode ? F("DAY TEST*  ") : F("DAY TEST   "));
  if (rtcOk) printClock(*lcd, n); else lcd->print(F("NO CLOCK"));
  lcd->setCursor(0, 1); lcd->print(F("Shows ")); lcd->print(showCount); lcd->print(F("  Faults ")); lcd->print(faultCount); lcd->print(F("    "));
  lcd->setCursor(0, 2);
  if (state >= CAL_SEEK_HOME && state <= CAL_PARK) lcd->print(F("Calibrating...      "));
  else if (!calibrated) lcd->print(F("Not calibrated      "));
  else { lcd->print(F("Next ")); if (rtcOk) printNext(*lcd, n); lcd->print(F("  Boot ")); lcd->print(bootCount); lcd->print(F("   ")); }
  lcd->setCursor(0, 3); lcd->print(F("Drift ")); lcd->print(lastDrift); lcd->print(F(" max ")); lcd->print(maxDrift); lcd->print(F(" st    "));
}
void printStatus() {
  DateTime n = rtcOk ? rtc.now() : DateTime();
  Serial.print(F("[")); if (rtcOk) printClock(Serial, n); Serial.print(F("] shows ")); Serial.print(showCount);
  Serial.print(F(" | faults ")); Serial.print(faultCount);
  if (faultCount) { Serial.print(F(" (last ")); printTwo(Serial, lastFaultH); Serial.print(':'); printTwo(Serial, lastFaultM); Serial.print(')'); }
  Serial.print(F(" | power-ups ")); Serial.print(bootCount);
  Serial.print(F(" | drift ")); Serial.print(lastDrift); Serial.print(F(" max ")); Serial.print(maxDrift);
  Serial.print(F(" | up ")); Serial.print(millis() / 3600000UL); Serial.print(F("h")); printTwo(Serial, (millis() / 60000UL) % 60);
  if (rtcOk) { Serial.print(F(" | next ")); printNext(Serial, n); }
  Serial.println(fastMode ? F(" | FAST MODE") : F(""));
}

void printSwitches() {
  Serial.print(F("HOME (D8): ")); Serial.print(homeHit ? F("HIT") : F("clear"));
  Serial.print(F("   FAR (D9): ")); Serial.println(farHit ? F("HIT") : F("clear"));
}

// ---- State changes ----
void stopAll(const __FlashStringHelper* msg, bool lostPosition) {
  while (dfIdx < 10) dfSerial.write(dfPkt[dfIdx++]);   // finish a half-sent command so the DFPlayer isn't left mid-packet
  stepper.setSpeed(0);
  stepper.disableOutputs();
  state = IDLE;
  countNow = 0;
  showAfterCal = false;
  if (lostPosition) calibrated = false;
  Serial.println(msg);
  updateLcd();
}
void fault(const __FlashStringHelper* msg) {
  faultCount++;
  if (rtcOk) { DateTime n = rtc.now(); lastFaultH = n.hour(); lastFaultM = n.minute(); Serial.print('['); printClock(Serial, n); Serial.print(F("] ")); }
  Serial.print(F("FAULT #")); Serial.print(faultCount); Serial.print(F(": "));
  stopAll(msg, true);
  player.stop();                       // don't let a cart clip thud after a fault stop
  printSwitches();
}

void startCalibration(bool thenShow) {
  configureDriver();
  stepper.setPinsInverted(dirInvert, false, true);
  stepper.enableOutputs();
  showAfterCal = thenShow;
  calibrated = false;
  stepper.setCurrentPosition(0);
  if (farHit && homeHit) { fault(F("both switches read HIT. Check wiring (type w).")); return; }
  if (homeHit) {                       // already sitting on HOME: back off first
    state = CAL_BACKOFF; markPos = entryPos = 0;
    Serial.println(F("Calibrating: on HOME already, backing off"));
  } else {
    state = CAL_SEEK_HOME;
    Serial.println(F("Calibrating: looking for HOME"));
  }
  updateLcd();
}

uint16_t extendLagNow = EXTEND_DF_LAG_MS;
bool bellSawBusy = false;
unsigned long bellDoneAt = 0;
void startCartAndExtend() {
  extendLagNow = EXTEND_DF_LAG_AFTER_BELL_MS;
  if (CART_SOUND) play(CART_TRACK);
  state = EXTEND_CUE;
  pauseStart = millis();
}

void startShow() {
  configureDriver();
  stepper.enableOutputs();
  finishRequested = false;
  countNow = 0;
  strokeInCount = 0;
  Serial.print(F("Show: extend, row ")); Serial.print(hourCount); Serial.println(F(", retract"));
  if (BELL_BEFORE_SHOW && soundOn) {   // bells first, ship waits until they have fully rung out
    play(BELL_TRACK);
    Serial.println(F("Bells"));
    bellSawBusy = false;
    bellDoneAt = 0;
    state = BELL;
    pauseStart = millis();
    return;
  }
  extendLagNow = EXTEND_DF_LAG_MS;
  if (CART_SOUND) play(CART_TRACK);    // sound first, then the move after the DFPlayer's start-up lag
  state = EXTEND_CUE;
  pauseStart = millis();
}

void beginRecover() { state = RECOVER; rowCueSent = false; startSegment(-ROW_STROKE, ROW_RECOVER_MS, RECOVER_CURVE); }
void beginDrive()   { state = DRIVE;   startSegment(ROW_STROKE, ROW_DRIVE_MS, DRIVE_CURVE); }
void beginRetract() {
  if (CART_SOUND && CART_ON_RETRACT) play(CART_TRACK);
  state = RETRACT_CUE;
  pauseStart = millis();
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '0' && c <= '9') {
    if (state == IDLE) { hourCount = (c == '0') ? 12 : c - '0'; Serial.print(F("Hour count: ")); Serial.println(hourCount); updateLcd(); }
    return;
  }
  switch (c) {
    case 'w': case 'W': printSwitches(); break;
    case 'g': case 'G':
      if (state != IDLE) { Serial.println(F("Already running")); break; }
      showCount++;
      if (rtcOk) { uint8_t h = rtc.now().hour(); hourCount = (h % 12 == 0) ? 12 : h % 12; }
      Serial.print(F("Manual show #")); Serial.println(showCount);
      if (calibrated) startShow(); else startCalibration(true);
      break;
    case 'k': case 'K':
      if (state == IDLE) startCalibration(false);
      break;
    case 'x': case 'X':
      if (state >= BELL && state <= REST) { finishRequested = true; Serial.println(F("Finishing after this stroke")); }
      break;
    case 'p': case 'P': case ' ':
      if (state != IDLE) { stopAll(F("Stopped now, motor released. Type k or g to recalibrate."), true); player.stop(); }
      break;
    case 'h': case 'H':
      if (state == IDLE) {
        if (!calibrated) { Serial.println(F("Not calibrated yet: type k")); break; }
        configureDriver(); stepper.enableOutputs();
        state = PARKING; startTravel(PARK_OFFSET, RETRACT_SPEED, true);
      }
      break;
    case 't': case 'T': player.playMp3Folder(ROW_TRACK); Serial.println(F("Playing /mp3/0001.mp3")); break;
    case 'b': case 'B': if (state == IDLE) { player.playMp3Folder(BELL_TRACK); Serial.println(F("Playing /mp3/0005.mp3")); } break;
    case 'c': case 'C': if (state == IDLE) { player.playMp3Folder(CART_TRACK); Serial.println(F("Playing /mp3/0002.mp3")); } break;
    case 'n': case 'N': soundOn = !soundOn; Serial.println(soundOn ? F("Sound ON") : F("Sound OFF")); break;
    case '+': volumeLevel = min(volumeLevel + 2, 30); player.volume(volumeLevel); Serial.print(F("Volume ")); Serial.println(volumeLevel); break;
    case '-': volumeLevel = max(volumeLevel - 2, 0);  player.volume(volumeLevel); Serial.print(F("Volume ")); Serial.println(volumeLevel); break;
    case 'm': case 'M':
      quietMode = !quietMode; driver.en_spreadCycle(!quietMode);
      Serial.println(quietMode ? F("Quiet mode (StealthChop)") : F("Strong mode (SpreadCycle)"));
      break;
    case 's': case 'S': printStatus(); break;
    case 'f': case 'F':
      if (state == IDLE) { fastMode = !fastMode; Serial.println(fastMode ? F("FAST MODE: show every 5 minutes") : F("Hourly mode: 9:00 to 20:00")); updateLcd(); }
      break;
    case 'z': case 'Z':
      bootCount = 0; EEPROM.update(1, 0); EEPROM.update(2, 0); Serial.println(F("Power-up counter cleared")); updateLcd();
      break;
    case 'r': case 'R':
      if (state == IDLE) { dirInvert = !dirInvert; calibrated = false; Serial.println(F("Motor direction flipped. Type k to calibrate.")); }
      break;
  }
}

void setup() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);
  pinMode(PIN_HOME, INPUT_PULLUP);
  pinMode(PIN_FAR, INPUT_PULLUP);
  pinMode(PIN_DF_BUSY, INPUT_PULLUP);

  Serial.begin(9600);
  delay(200);
  Serial.println(F("\n=== Viking Ship: DAY TEST (shows 9:00 to 20:00 on the hour) ==="));
  if (EEPROM.read(0) != 0x5A) { EEPROM.update(0, 0x5A); EEPROM.update(1, 0); EEPROM.update(2, 0); }
  bootCount = (EEPROM.read(1) | (EEPROM.read(2) << 8)) + 1;
  EEPROM.update(1, bootCount & 0xFF); EEPROM.update(2, bootCount >> 8);
  Serial.print(F("Power-up #")); Serial.println(bootCount);

  tmcSerial.begin(19200);
  dfSerial.begin(9600);
  delay(1500);
  player.begin(dfSerial, false, false);
  delay(200);
  player.volume(volumeLevel);
  configureDriver();

  Wire.begin();
  uint8_t a = findLcd();
  if (a) { lcd = new LiquidCrystal_I2C(a, 20, 4); lcd->init(); lcd->backlight(); }
  rtcOk = rtc.begin();

  stepper.setEnablePin(PIN_EN);
  stepper.setPinsInverted(dirInvert, false, true);
  stepper.disableOutputs();
  stepper.setMaxSpeed(MAX_STEP_RATE);

  for (byte i = 0; i < 20; i++) { updateSwitches(); delay(1); }
  Serial.println(F("s status | f fast mode | z clear power-ups | g show now | k calibrate | x finish | p stop | h park | t/c/b sounds | n sound | +/- vol | w switches"));
  if (!rtcOk) Serial.println(F("WARNING: no DS3231 clock found, the schedule can't run (g still works)."));
  printSwitches();
  updateLcd();

  if (AUTO_CALIBRATE) {
    Serial.println(F("Power-up calibration starting (type p to stop)"));
    delay(STARTUP_DELAY_MS);
    for (byte i = 0; i < 20; i++) { updateSwitches(); delay(1); }
    startCalibration(false);
  } else {
    Serial.println(F("Type k to calibrate or g for the show"));
  }
}

void loop() {
  handleSerial();
  updateSwitches();
  serviceDf();

  // Safety: during the show, any switch hit is a fault (park and extend stop short of both)
  if (state >= BELL && (homeHit || farHit)) { fault(F("limit switch hit during the show.")); return; }

  switch (state) {
    case IDLE: {
      static unsigned long lastLcd = 0;
      if (millis() - lastLcd > 1000) {
        lastLcd = millis();
        updateLcd();
        if (rtcOk) {   // schedule check, once a second while parked
          DateTime n = rtc.now();
          uint8_t h = n.hour(), m = n.minute();
          bool due = inWindow(h) && n.second() < 30 && (fastMode ? (m % 5 == 0) : (m == 0));
          long key = (long)n.day() * 1440L + h * 60L + m;
          if (due && key != lastShowKey) {
            lastShowKey = key;
            hourCount = (h % 12 == 0) ? 12 : h % 12;
            showCount++;
            Serial.print('['); printClock(Serial, n); Serial.print(F("] Show #")); Serial.print(showCount);
            Serial.print(F(", rowing ")); Serial.println(hourCount);
            if (calibrated) startShow(); else startCalibration(true);
          }
        }
      }
      break;
    }

    // ---- Calibration (same routine as travel_test.ino) ----
    case CAL_SEEK_HOME:
      if (farHit)  { fault(F("hit FAR while looking for HOME. Motor runs backwards: type r, then k.")); break; }
      if (homeHit) { state = CAL_BACKOFF; markPos = entryPos = stepper.currentPosition(); break; }
      if (labs(stepper.currentPosition()) > MAX_SEEK_STEPS) { fault(F("HOME never tripped (40\" limit). Is the 12V on? Check the switch, then k.")); break; }
      stepper.setSpeed(-SEEK_SPEED); stepper.runSpeed();
      break;

    case CAL_BACKOFF:   // out until HOME releases, then HOME_BACKOFF more
      if (farHit) { fault(F("hit FAR while backing off HOME.")); break; }
      if (homeHit) {
        markPos = stepper.currentPosition();
        if (markPos - entryPos > 677) { fault(F("HOME never released after 2\". Check the switch.")); break; }
      } else if (stepper.currentPosition() - markPos >= HOME_BACKOFF) {
        markPos = stepper.currentPosition();
        state = CAL_SLOW_HOME;
        break;
      }
      stepper.setSpeed(SEEK_SPEED); stepper.runSpeed();
      break;

    case CAL_SLOW_HOME: // touch HOME again slowly for a precise zero
      if (homeHit) {
        stepper.setCurrentPosition(0);
        state = CAL_SEEK_FAR;
        Serial.println(F("HOME set. Looking for FAR"));
        break;
      }
      if (markPos - stepper.currentPosition() > HOME_BACKOFF * 3) { fault(F("lost HOME on the slow touch.")); break; }
      stepper.setSpeed(-SLOW_SEEK_SPEED); stepper.runSpeed();
      break;

    case CAL_SEEK_FAR:
      if (farHit) {
        farTrip = stepper.currentPosition();
        extendPos = farTrip - FAR_MARGIN;
        Serial.print(F("FAR found. Travel: ")); Serial.print(farTrip / 338.67, 2);
        Serial.print(F(" in (")); Serial.print(farTrip); Serial.println(F(" steps)"));
        Serial.print(F("Extend stops at ")); Serial.print(extendPos / 338.67, 2);
        Serial.print(F(" in. Timed extend/retract cruise ~"));
        Serial.print(speedForTime(extendPos - PARK_OFFSET, CART_THUD_MS) / 338.67, 2); Serial.println(F(" in/s"));
        if (extendPos - ROW_STROKE < PARK_OFFSET + 85) { fault(F("travel is too short for a 7\" stroke.")); break; }
        calibrated = true;
        state = CAL_PARK;
        startTravel(PARK_OFFSET, RETRACT_SPEED, true);
        break;
      }
      if (stepper.currentPosition() > MAX_SEEK_STEPS) { fault(F("FAR never tripped (40\" limit). Check the switch and wiring.")); break; }
      if (homeHit && stepper.currentPosition() > HOME_BACKOFF) { fault(F("HOME tripped on the way to FAR. Check wiring.")); break; }
      stepper.setSpeed(SEEK_SPEED); stepper.runSpeed();
      break;

    case CAL_PARK:      // leaving FAR, so ignore FAR for the first inch
      if (farHit && stepper.currentPosition() < farTrip - 339) { fault(F("FAR still reads HIT after leaving it.")); break; }
      if (homeHit && stepper.currentPosition() > HOME_BACKOFF) { fault(F("HOME hit while parking.")); break; }
      if (runTravel()) {
        if (showAfterCal) { showAfterCal = false; startShow(); }
        else stopAll(F("Calibrated and parked. Type g for the show."), false);
      }
      break;

    // ---- Show ----
    case BELL: {        // ship stays parked until the bells have fully rung out
      unsigned long el = millis() - pauseStart;
      bool done;
      if (USE_BUSY_PIN) {
        bool busy = digitalRead(PIN_DF_BUSY) == LOW;
        if (busy) bellSawBusy = true;
        done = (bellSawBusy && !busy && el > 300) || el > (unsigned long)BELL_MS + 3000;  // ended, or safety timeout
      } else {
        done = el >= BELL_MS;
      }
      if (done && bellDoneAt == 0) bellDoneAt = millis();
      if (bellDoneAt && millis() - bellDoneAt >= BELL_GAP_MS) { bellDoneAt = 0; startCartAndExtend(); }
      break;
    }
    case EXTEND_CUE:    // cart sound was sent; wait out the DFPlayer's start-up lag, then go
      if (millis() - pauseStart >= extendLagNow) { state = EXTEND; startTimedTravel(extendPos, true); }
      break;
    case EXTEND:
      if (runTravel()) { reportTimed(F("Extend")); state = HOLD_IN; pauseStart = millis(); }
      break;
    case HOLD_IN:
      if (millis() - pauseStart >= HOLD_BEFORE_ROW_MS) { countNow = 1; beginRecover(); }
      break;
    case RECOVER:   // drum + "Row!" starts ROW_VOICE_OFFSET_MS before the catch, so "Row!" lands where it used to
      if (!rowCueSent && ROW_VOICE_OFFSET_MS > 0 &&
          millis() - segStart >= (unsigned long)(ROW_RECOVER_MS - min(ROW_VOICE_OFFSET_MS, ROW_RECOVER_MS))) {
        queuePlay(ROW_TRACK); rowCueSent = true;
      }
      if (runSegment()) {
        state = CATCH; pauseStart = millis();
        if (!rowCueSent) { play(ROW_TRACK); rowCueSent = true; }   // offset 0: send at the catch, motor stopped
      }
      break;
    case CATCH:
      if (millis() - pauseStart >= CATCH_PAUSE_MS) beginDrive();
      break;
    case DRIVE:
      if (runSegment()) { state = FINISH; pauseStart = millis(); }
      break;
    case FINISH:
      if (millis() - pauseStart >= FINISH_PAUSE_MS) {
        strokeInCount++;
        bool lastCount = countNow >= hourCount;
        if (!finishRequested && strokeInCount < STROKES_PER_COUNT) { beginRecover(); break; }
        strokeInCount = 0;
        if (finishRequested || lastCount) { state = HOLD_OUT; pauseStart = millis(); }
        else { state = REST; pauseStart = millis(); }
      }
      break;
    case REST:
      if (millis() - pauseStart >= COUNT_PAUSE_MS) { countNow++; beginRecover(); }
      break;
    case HOLD_OUT:
      if (millis() - pauseStart >= HOLD_AFTER_ROW_MS) beginRetract();
      break;
    case RETRACT_CUE:
      if (millis() - pauseStart >= RETRACT_DF_LAG_MS) { state = RETRACT; startTimedTravel(PARK_OFFSET, false); }
      break;
    case RETRACT:
      if (runTravel()) { reportTimed(F("Retract")); state = DRIFT_SEEK; }
      break;
    case DRIFT_SEEK:    // creep back onto HOME: where it trips shows any lost steps
      if (farHit) { fault(F("FAR hit during the drift check.")); break; }
      if (homeHit) {
        lastDrift = stepper.currentPosition();
        if (labs(lastDrift) > labs(maxDrift)) maxDrift = lastDrift;
        stepper.setCurrentPosition(0);          // re-zero on the fresh touch
        state = DRIFT_PARK;
        startTravel(PARK_OFFSET, RETRACT_SPEED, false);
        break;
      }
      if (stepper.currentPosition() < -339) { fault(F("HOME didn't trip within 1\" past zero: lost steps or switch problem.")); break; }
      stepper.setSpeed(-SLOW_SEEK_SPEED); stepper.runSpeed();
      break;
    case DRIFT_PARK:
      if (farHit) { fault(F("FAR hit while parking.")); break; }
      if (homeHit && stepper.currentPosition() > HOME_BACKOFF) { fault(F("HOME still hit after parking.")); break; }
      if (runTravel()) {
        Serial.print(F("Show #")); Serial.print(showCount); Serial.print(F(" done: rowed ")); Serial.print(hourCount);
        Serial.print(F(" | extend ")); Serial.print(lastExtendMs / 1000.0, 2); Serial.print(F(" s | retract ")); Serial.print(lastRetractMs / 1000.0, 2);
        Serial.print(F(" s | drift ")); Serial.print(lastDrift); Serial.println(F(" steps"));
        printStatus();
        stopAll(F("Parked."), false);
      }
      break;
    case PARKING:
      if (runTravel()) stopAll(F("Parked."), false);
      break;
  }
}
