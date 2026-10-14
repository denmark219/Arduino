// kRINGle Doorbell, bench test #4: doorbell + test mode + "kRINGle" nameplate backlight
//
// NAMEPLATE:   SK6812 RGBWW strip behind the letters, almost-yellow warm white with a
//              subtle, uneven waver, like an old bulb on slightly unsteady power.
//
// NORMAL MODE: ring idles soft white. A press turns it red and plays the ding dong,
//              then it returns to white after RED_HOLD_MS.
// TEST MODE:   press the button 5 times within 3 seconds to enter. The ring loops:
//                1) primaries: red, green, blue, then white (1s each)
//                2) full color sweep around the color wheel (6s)
//                3) rainbow chase around the ring (5s)
//              The nameplate joins in (red, green, blue, its own warm-white LED,
//              color sweep, chase). Press the button once to exit back to normal mode.
//
// Board: Arduino Nano ESP32 (Arduino ESP32 Boards package)
// IMPORTANT: Tools -> Pin Numbering -> "By GPIO number (legacy)"
//
// Wiring: same as bench test #2 (see the kRINGle Ring Bench Wiring page)
//   Switch NC -> D2, COM -> GND      Ring DI -> D5, 5V, GND
//   DFPlayer RX -> D4, VCC -> 5V, GND, TX unconnected, SPK_1/SPK_2 -> speaker
//   Nameplate strip DIN -> D6, 5V -> 5V, GND -> GND  (5V version of the strip only!)
//
// Libraries: Adafruit NeoPixel, DFRobotDFPlayerMini

#include <Adafruit_NeoPixel.h>
#include <DFRobotDFPlayerMini.h>

#define RING_PIN       D5
#define RING_PIXELS    24
#define BUTTON_PIN     D2
#define DF_TX_PIN      D4
#define DF_RX_PIN      D3     // unused, DFPlayer TX not wired
#define NAME_PIN       D6
#define NAME_PIXELS    8      // count the pixels on your cut piece (~5" at 60/m = 7 or 8)

#define PRESSED_LEVEL  HIGH   // NC wiring. Use LOW if moved to NO.

#define BRIGHTNESS     40
#define DF_VOLUME      20
#define DING_TRACK     1

// ---- Nameplate look ----
#define NAME_BRIGHTNESS 90    // overall nameplate brightness, 0-255
// Almost-yellow warm white: the warm-white LED plus a push of red and green, no blue.
// More G = more yellow, more R = more amber/orange, more W = paler.
const uint8_t NAME_R = 110, NAME_G = 55, NAME_B = 0, NAME_W = 200;

// Waver: each pixel drifts on its own between WAVER_MIN and 100% brightness.
const float         WAVER_MIN        = 0.88;  // 0.88 = dips up to 12%. Closer to 1.0 = subtler
const unsigned long WAVER_MIN_MS     = 120;   // how often each pixel picks a new level
const unsigned long WAVER_MAX_MS     = 450;
const float         WAVER_SMOOTH     = 0.12;  // 0-1, lower = slower, softer drift
// Occasional "power sag": the whole strip briefly dims together, now and then.
const float         SAG_LEVEL        = 0.80;  // brightness during a sag
const unsigned long SAG_MS           = 90;
const int           SAG_CHANCE_PER_S = 6;     // percent chance per second (0 = never)

// Test mode entry: this many presses within this window
const int           TEST_PRESSES   = 5;
const unsigned long TEST_WINDOW_MS = 3000;

const unsigned long DEBOUNCE_MS   = 30;
const unsigned long RED_HOLD_MS   = 3000;
const unsigned long EXIT_LOCK_MS  = 600;   // ignore presses briefly after leaving test mode

// Test sequence timing
const unsigned long PRIMARY_MS    = 1000;  // per color
const unsigned long SWEEP_MS      = 6000;
const unsigned long CHASE_MS      = 5000;
const unsigned long CHASE_STEP_MS = 45;    // lower = faster chase
const int           CHASE_TAIL    = 6;     // pixels in the fading tail
const unsigned long FRAME_MS      = 20;    // ~50 updates per second

Adafruit_NeoPixel ring(RING_PIXELS, RING_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel plate(NAME_PIXELS, NAME_PIN, NEO_GRBW + NEO_KHZ800);
DFRobotDFPlayerMini doorbellPlayer;

// Nameplate waver state
float nameLevel[NAME_PIXELS];
float nameTarget[NAME_PIXELS];
unsigned long nameNextPick[NAME_PIXELS];
unsigned long sagUntil = 0;
unsigned long lastSagRoll = 0;
unsigned long lastNameFrameAt = 0;

const uint32_t SOFT_WHITE = Adafruit_NeoPixel::Color(255, 190, 120);
const uint32_t RED        = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t PRIMARIES[] = {
  Adafruit_NeoPixel::Color(255, 0, 0),     // red
  Adafruit_NeoPixel::Color(0, 255, 0),     // green
  Adafruit_NeoPixel::Color(0, 0, 255),     // blue
  Adafruit_NeoPixel::Color(255, 255, 255)  // white (all three)
};
const int NUM_PRIMARIES = 4;

enum Mode { NORMAL, TEST };
Mode mode = NORMAL;

// Normal mode state
bool isRed = false;
unsigned long redStartedAt = 0;

// Button state
int lastReading, stableState;
unsigned long lastChangeAt = 0;
unsigned long ignoreUntil = 0;

// Press counting for test mode entry
int pressCount = 0;
unsigned long firstPressAt = 0;

// Test mode state
unsigned long testStartedAt = 0;
unsigned long lastFrameAt = 0;
const unsigned long CYCLE_MS = PRIMARY_MS * NUM_PRIMARIES + SWEEP_MS + CHASE_MS;

void fillRing(uint32_t c) {
  ring.fill(c);
  ring.show();
}

// ---------- Nameplate ----------
void drawNameplateNormal(unsigned long now) {
  // Roll once a second for an occasional whole-strip sag
  if (now - lastSagRoll >= 1000) {
    lastSagRoll = now;
    if (random(100) < SAG_CHANCE_PER_S) sagUntil = now + SAG_MS;
  }
  float sag = (now < sagUntil) ? SAG_LEVEL : 1.0;

  for (int i = 0; i < NAME_PIXELS; i++) {
    if (now >= nameNextPick[i]) {
      nameTarget[i] = WAVER_MIN + (1.0 - WAVER_MIN) * (random(1000) / 1000.0);
      nameNextPick[i] = now + random(WAVER_MIN_MS, WAVER_MAX_MS);
    }
    nameLevel[i] += (nameTarget[i] - nameLevel[i]) * WAVER_SMOOTH;
    float k = nameLevel[i] * sag;
    plate.setPixelColor(i, plate.Color((uint8_t)(NAME_R * k), (uint8_t)(NAME_G * k), (uint8_t)(NAME_B * k), (uint8_t)(NAME_W * k)));
  }
  plate.show();
}

// ---------- Test pattern ----------
void drawTestFrame(unsigned long t) {
  unsigned long pos = t % CYCLE_MS;   // where we are in the looping sequence

  if (pos < PRIMARY_MS * NUM_PRIMARIES) {
    // 1) Primaries, one solid color per second
    int idx = pos / PRIMARY_MS;
    ring.fill(PRIMARIES[idx]);
    // Nameplate: red, green, blue, then its dedicated warm-white LED only
    const uint32_t namePrim[] = { plate.Color(255,0,0,0), plate.Color(0,255,0,0),
                                  plate.Color(0,0,255,0), plate.Color(0,0,0,255) };
    plate.fill(namePrim[idx]);

  } else if (pos < PRIMARY_MS * NUM_PRIMARIES + SWEEP_MS) {
    // 2) Whole ring sweeps once around the color wheel
    unsigned long s = pos - PRIMARY_MS * NUM_PRIMARIES;
    uint16_t hue = (uint32_t)s * 65536UL / SWEEP_MS;
    ring.fill(ring.gamma32(ring.ColorHSV(hue)));
    plate.fill(plate.gamma32(plate.ColorHSV(hue)));

  } else {
    // 3) Rainbow chase: a bright head with a fading tail, color shifts each step
    unsigned long c = pos - PRIMARY_MS * NUM_PRIMARIES - SWEEP_MS;
    unsigned long step = c / CHASE_STEP_MS;
    int head = step % RING_PIXELS;
    uint16_t hue = (step * 65536UL / (RING_PIXELS * 2)) & 0xFFFF;
    ring.clear();
    for (int i = 0; i < CHASE_TAIL; i++) {
      int p = (head - i + RING_PIXELS) % RING_PIXELS;
      uint8_t v = 255 - i * (230 / CHASE_TAIL);   // dimmer toward the tail
      ring.setPixelColor(p, ring.gamma32(ring.ColorHSV(hue, 255, v)));
    }
    // Nameplate: a short chase of its own along the letters
    plate.clear();
    int nh = step % NAME_PIXELS;
    plate.setPixelColor(nh, plate.gamma32(plate.ColorHSV(hue)));
    plate.setPixelColor((nh + NAME_PIXELS - 1) % NAME_PIXELS, plate.gamma32(plate.ColorHSV(hue, 255, 90)));
  }
  ring.show();
  plate.show();
}

void enterTestMode() {
  mode = TEST;
  isRed = false;
  pressCount = 0;
  testStartedAt = millis();
  lastFrameAt = 0;
  Serial.println("TEST MODE: press once to exit");
}

void exitTestMode() {
  mode = NORMAL;
  pressCount = 0;
  ignoreUntil = millis() + EXIT_LOCK_MS;
  fillRing(SOFT_WHITE);
  Serial.println("Back to normal mode");
}

// ---------- Button handling ----------
void onPress() {
  unsigned long now = millis();

  if (mode == TEST) {
    exitTestMode();
    return;
  }

  // Count presses toward test mode entry
  if (pressCount == 0 || now - firstPressAt > TEST_WINDOW_MS) {
    pressCount = 1;
    firstPressAt = now;
  } else {
    pressCount++;
  }
  if (pressCount >= TEST_PRESSES) {
    enterTestMode();
    return;
  }

  // Normal doorbell (presses while red are counted but don't re-ring)
  if (!isRed) {
    Serial.println("Pressed: ding dong");
    fillRing(RED);
    doorbellPlayer.playMp3Folder(DING_TRACK);
    isRed = true;
    redStartedAt = now;
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastReading = stableState = digitalRead(BUTTON_PIN);

  ring.begin();
  ring.setBrightness(BRIGHTNESS);
  fillRing(SOFT_WHITE);

  plate.begin();
  plate.setBrightness(NAME_BRIGHTNESS);
  randomSeed(esp_random());
  for (int i = 0; i < NAME_PIXELS; i++) {
    nameLevel[i] = nameTarget[i] = 1.0;
    nameNextPick[i] = 0;
  }

  Serial1.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  delay(1500);
  doorbellPlayer.begin(Serial1, false, false);
  delay(200);
  doorbellPlayer.volume(DF_VOLUME);

  Serial.println("Doorbell ready. 5 quick presses = test mode");
}

void loop() {
  unsigned long now = millis();

  // Debounced press detection
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    lastChangeAt = now;
    lastReading = reading;
  }
  if (now - lastChangeAt > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == PRESSED_LEVEL && now >= ignoreUntil) {
      onPress();
    }
  }

  if (mode == NORMAL) {
    if (now - lastNameFrameAt >= FRAME_MS) {
      lastNameFrameAt = now;
      drawNameplateNormal(now);
    }
    if (isRed && now - redStartedAt >= RED_HOLD_MS) {
      fillRing(SOFT_WHITE);
      isRed = false;
    }
  } else {
    if (now - lastFrameAt >= FRAME_MS) {
      lastFrameAt = now;
      drawTestFrame(now - testStartedAt);
    }
  }
}
