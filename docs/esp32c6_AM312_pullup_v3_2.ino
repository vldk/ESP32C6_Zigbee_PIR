/*
 * Zigbee battery PIR presence sensor
 * ----------------------------------
 * Board : Seeed XIAO ESP32-C6
 * Sensor: AM312 mini PIR  ->  NPN buffer  ->  D1/GPIO1 (LP/RTC pin, wakes deep sleep)
 * Power : LiPo on BAT pads, battery sense via 2x1MegOhm divider -> D0/GPIO0
 *
 * ---- Battery reporting ----
 * Percentage: reported via the Power-Config cluster (occupancy endpoint 10).
 * Voltage   : the standard Power-Config BatteryVoltage attribute (0x0020) is
 *   READ-ONLY / NON-REPORTABLE in this library (z2m configReport returns
 *   UNREPORTABLE_ATTRIBUTE), so we CANNOT push it. Instead we report the battery
 *   voltage (in volts) through an Analog Input cluster on a 2nd endpoint (11),
 *   which IS reportable. z2m maps genAnalogInput.presentValue -> "voltage".
 *   Both are sent as UNSOLICITED reports each wake, so no bind / configureReporting
 *   is needed (those time out on a sleepy end device anyway).
 *
 * ---- Configurable occupancy hold (NEW) ----
 * OCCUPANCY_HOLD_S is now the DEFAULT only. The live value is stored in NVS and
 * is settable from z2m via an Analog Output cluster on a 3rd endpoint (12).
 * z2m writes genAnalogOutput.presentValue; the device receives it during its
 * awake window (it polls its parent for the queued write), clamps + saves it to
 * NVS, and echoes it back so z2m confirms the accepted value.
 *   NOTE: because this is a sleepy end device, a change you make in z2m is queued
 *   and only lands the NEXT time the sensor wakes (motion, or the heartbeat), and
 *   only takes effect from the wake AFTER that. Allow up to one HEARTBEAT_S.
 *
 * ---- Recovery note ----
 * If Zigbee.begin() fails we DEEP-SLEEP and retry instead of ESP.restart()
 * (a software reset does not reliably re-init the C6's 802.15.4 radio).
 *
 * ---- Factory reset / re-pair (GPIO7 button, hold 5s) ----
 * Wire an external momentary switch from GPIO7 (D7) to GND. Hold it to GND for
 * RESET_HOLD_S seconds to leave the Zigbee network and reboot into pairing mode.
 * This works in EVERY state:
 *   - Sleeping : GPIO7 is a deep-sleep wake source, so pressing the button wakes
 *                the device; keep holding 5s and it factory-resets.
 *   - Awake but not joined (stuck connecting): the connect loop polls the button.
 *   - Cold boot / interview window: also polled.
 * GPIO7 is an LP/RTC pin (GPIO0-7), required for deep-sleep GPIO wakeup. Its
 * internal pull-up is auto-enabled during deep sleep (idle = HIGH), so no external
 * pull-up is needed; add a 1M to 3V3 if you want extra noise margin.
 * NOTE: GPIO7 has no ONBOARD button (the physical BOOT button is the GPIO9 strap
 * pin, unusable here). Wire an external momentary switch to GND on GPIO7 to use it.
 *
 * ---- Why the buffer transistor ----
 *   AM312 OUT --[100k]--> Base (2N3904 / BC547 / BC337)
 *                         Emitter --> GND
 *                         Collector --> D1 / GPIO1
 *   D1 / GPIO1 --[1M]--> 3V3           (pull-up: holds the line HIGH when idle)
 *   No motion : AM312 OUT = 0V   -> NPN off -> D1 = HIGH
 *   Motion    : AM312 OUT = 1.8V -> NPN on  -> D1 = LOW   <- wake on this
 *
 * ---- Arduino IDE settings (Tools menu) ----
 *   Board            : XIAO_ESP32C6   (ESP32 board package 3.x)
 *   Zigbee Mode      : Zigbee ED (end device)
 *   Partition Scheme : Zigbee 4MB with spiffs
 *   Erase All Flash  : DISABLED once joined (Enabled only to wipe & re-pair)
 */

#ifndef ZIGBEE_MODE_ED
#error "Select Tools > Zigbee Mode > Zigbee ED (end device) before compiling."
#endif

#include "Zigbee.h"
#include "esp_sleep.h"
#include <Preferences.h>   // NVS-backed key/value store (survives power loss)

// ---------------- Pins ----------------
#define PIR_PIN        1     // D1 / GPIO1  (buffered PIR signal) - LP/RTC pin (GPIO0-7)
#define BATTERY_PIN    0     // D0 / GPIO0  (ADC on the divider mid-point)
#define BOOT_BUTTON    7     // GPIO7 (D7) -> external button to GND. Hold RESET_HOLD_S to
                             // factory-reset + re-pair. LP/RTC pin so it can wake deep sleep.

#define ZB_JOIN_RETRY_S  30  // if the stack won't start/join, sleep this long then retry
#define RESET_HOLD_S     5   // hold the GPIO7 button to GND continuously this long -> factory reset

// Built-in user LED on the XIAO ESP32-C6 is GPIO15 and is ACTIVE-LOW (LOW = lit).
#define LED_PIN          15
#define LED_ON           LOW
#define LED_OFF          HIGH

// The NPN buffer inverts the signal: motion pulls the pin LOW, idle is HIGH.
#define MOTION_LEVEL   LOW    // digitalRead(PIR_PIN) == MOTION_LEVEL  means "motion"

// ---------------- Behaviour ----------------
#define OCCUPANCY_HOLD_DEFAULT_S  30          // DEFAULT stay-occupied time (now configurable via z2m)
#define OCCUPANCY_HOLD_MIN_S      1           // clamp: refuse anything shorter (1s = report even brief motion)
#define OCCUPANCY_HOLD_MAX_S      3600        // clamp: refuse anything longer (1 h)
#define HEARTBEAT_S          (60 * 60)   // wake at least this often (battery + keepalive)
#define MAX_AWAKE_S          180         // safety: force clear+sleep after this even if motion stays asserted
#define CONFIG_LISTEN_S      1           // AFTER occupancy is reported, stay awake this long so the stack can
                                         // poll the parent for a queued genAnalogOutput (config) write from z2m.
                                         // It runs post-occupancy, so it never delays a motion report -- raise
                                         // it for more reliable config reception on heartbeat wakes if needed.

// ---------------- Battery divider ----------------
#define DIV_RATIO      2.0f    // 2x1MegOhm -> divide by 2
#define VBAT_MIN_MV    3000.0f // 0%   (~empty LiPo)
#define VBAT_MAX_MV    4200.0f // 100% (full LiPo)

// ---------------- Zigbee endpoints ----------------
#define OCCUPANCY_ENDPOINT   10
#define ANALOG_ENDPOINT      11
#define CONFIG_ENDPOINT      12
ZigbeeOccupancySensor zbOccupancy = ZigbeeOccupancySensor(OCCUPANCY_ENDPOINT);
ZigbeeAnalog          zbBattV     = ZigbeeAnalog(ANALOG_ENDPOINT);   // battery voltage (V)
ZigbeeAnalog          zbConfig    = ZigbeeAnalog(CONFIG_ENDPOINT);   // occupancy-hold setpoint (s)

// ---------------- Configurable state ----------------
Preferences prefs;
uint32_t occupancyHoldS = OCCUPANCY_HOLD_DEFAULT_S;   // live value, loaded from NVS in setup()

// Set from the Zigbee-stack callback context; consumed in setup(). Kept tiny/volatile
// so we do NOT do NVS writes or ZCL reports inside the stack callback itself.
volatile bool  g_holdPending    = false;
volatile float g_holdPendingVal = 0.0f;

// -------------------------------------------------
inline bool motionPresent() {
  return digitalRead(PIR_PIN) == MOTION_LEVEL;
}

// Visual confirmation that a factory reset was accepted: blink the built-in LED
// 5 times over ~3 seconds (300 ms on / 300 ms off).
void blinkResetIndication() {
  // pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, LED_ON);   delay(300);
    digitalWrite(LED_PIN, LED_OFF);  delay(300);
  }
}

// Leave the network and reboot into pairing mode. Safe to call before or after
// Zigbee.begin(). factoryReset(restart=true) erases network data and reboots, so
// this never returns.
void doFactoryReset() {
  Serial.println("Reset button held -> blinking LED, then Zigbee factory reset (leaving network). Rebooting to re-pair.");
  Serial.flush();
  blinkResetIndication();   // 5 blinks over ~3s so the user knows the reset registered
  Zigbee.factoryReset();    // erases network data and reboots (restart defaults to true)
  // not reached
}

// True only if the GPIO7 button is held to GND continuously for RESET_HOLD_S.
// Returns instantly (one read) when the button is not pressed, so it is cheap to
// poll inside the various awake loops. Blocks up to RESET_HOLD_S while held.
bool resetButtonHeld() {
  if (digitalRead(BOOT_BUTTON) != LOW) return false;   // not pressed
  uint32_t t0 = millis();
  while (digitalRead(BOOT_BUTTON) == LOW) {
    if (millis() - t0 >= (uint32_t)RESET_HOLD_S * 1000UL) return true;
    delay(20);
  }
  return false;   // released before the threshold
}

uint32_t clampHold(uint32_t s) {
  if (s < OCCUPANCY_HOLD_MIN_S) return OCCUPANCY_HOLD_MIN_S;
  if (s > OCCUPANCY_HOLD_MAX_S) return OCCUPANCY_HOLD_MAX_S;
  return s;
}

// z2m wrote genAnalogOutput.presentValue on the config endpoint. Runs in the Zigbee
// task context -> just latch the value; the real work happens in applyPendingHold().
void onHoldChange(float v) {
  g_holdPendingVal = v;
  g_holdPending    = true;
}

// If a new hold value arrived, clamp it, persist to NVS (only when it actually
// changed), and echo the accepted value back so z2m shows the real setting.
void applyPendingHold() {
  if (!g_holdPending) return;
  g_holdPending = false;

  uint32_t v = clampHold((uint32_t)(g_holdPendingVal + 0.5f));
  if (v != occupancyHoldS) {
    occupancyHoldS = v;
    prefs.begin("pir", false);
    prefs.putUInt("hold_s", occupancyHoldS);
    prefs.end();
    Serial.printf("Occupancy hold updated -> %u s (saved to NVS)\n", occupancyHoldS);
  }
  // Echo the (possibly clamped) value so z2m confirms what the device actually uses.
  zbConfig.setAnalogOutput((float)occupancyHoldS);
  zbConfig.reportAnalogOutput();
}

// Raw battery millivolts (averaged).
uint16_t readBatteryMilliVolts() {
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) { acc += analogReadMilliVolts(BATTERY_PIN); delay(2); }
  return (uint16_t)((acc / 8.0f) * DIV_RATIO);
}

uint8_t mvToPercent(uint16_t mv) {
  float pct = ((float)mv - VBAT_MIN_MV) * 100.0f / (VBAT_MAX_MV - VBAT_MIN_MV);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)(pct + 0.5f);
}

void goToDeepSleep() {
  // Wake on: buffered motion (GPIO1 LOW) OR the reset button (GPIO7 LOW) + periodic heartbeat.
  // Both pins wake on the same LOW level, so they share one mask. The internal pull-up on each
  // wake pin is auto-enabled during deep sleep (default ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS),
  // so an unpressed button idles HIGH and does not spuriously wake the device.
  esp_deep_sleep_enable_gpio_wakeup((1ULL << PIR_PIN) | (1ULL << BOOT_BUTTON), ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_sleep_enable_timer_wakeup((uint64_t)HEARTBEAT_S * 1000000ULL);
  Serial.println("Sleeping...");
  Serial.flush();
  esp_deep_sleep_start();   // never returns; MCU restarts on wake
}

void setup() {
  Serial.begin(115200);

  // Load the live occupancy-hold value from NVS (falls back to the compile default).
  prefs.begin("pir", true);                                     // read-only open
  occupancyHoldS = clampHold(prefs.getUInt("hold_s", OCCUPANCY_HOLD_DEFAULT_S));
  prefs.end();

  // External 1M pull-up sets the idle HIGH level; internal pull-up is a harmless backup.
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  delayMicroseconds(50);   // let the pull-up settle after reconfiguring the pad

  // Reset request: the GPIO7 button is a deep-sleep wake source, so pressing it while the
  // device sleeps wakes us here. If it is held to GND for RESET_HOLD_S we factory-reset and
  // reboot into pairing. This runs before Zigbee.begin() so it also works on a cold boot.
  // Reading the button now also lets us tell a button wake from a motion wake below: both
  // pull a GPIO LOW, so a raw GPIO wake cause is ambiguous on its own.
  bool btnLowAtWake = (digitalRead(BOOT_BUTTON) == LOW);
  if (btnLowAtWake) {
    if (resetButtonHeld()) doFactoryReset();   // reboots, never returns
    // Released before RESET_HOLD_S -> a button tap, NOT motion. btnLowAtWake stays true so
    // the occupancy logic below does not misread this wake as a motion event.
  }

  // Read battery once, up front.
  uint16_t mv   = readBatteryMilliVolts();
  uint8_t  pct  = mvToPercent(mv);

  // --- Endpoint 10: occupancy + battery percentage ---
  zbOccupancy.setManufacturerAndModel("DIY", "AM312_Presence_v2");
  zbOccupancy.setPowerSource(ZB_POWER_SOURCE_BATTERY, pct);   // percentage only

  // --- Endpoint 11: battery voltage as an Analog Input (reportable) ---
  zbBattV.addAnalogInput();
  zbBattV.setAnalogInputDescription("Battery Voltage");
  zbBattV.setAnalogInputResolution(0.001f);   // 1 mV resolution
  // Configure device-side reporting so the stack actually emits presentValue:
  //   min 0s, max = heartbeat, report on any change >= 0.005 V.
  zbBattV.setAnalogInputReporting(0, HEARTBEAT_S, 0.005f);

  // --- Endpoint 12: occupancy-hold setpoint as an Analog Output (writable from z2m) ---
  zbConfig.addAnalogOutput();
  zbConfig.setAnalogOutputDescription("Occupancy hold (s)");
  zbConfig.setAnalogOutputResolution(1.0f);                                  // whole seconds
  zbConfig.setAnalogOutputMinMax(OCCUPANCY_HOLD_MIN_S, OCCUPANCY_HOLD_MAX_S);// ZCL-level bounds hint
  zbConfig.onAnalogOutputChange(onHoldChange);                              // z2m writes land here

  Zigbee.addEndpoint(&zbOccupancy);
  Zigbee.addEndpoint(&zbBattV);
  Zigbee.addEndpoint(&zbConfig);
  Zigbee.setRxOnWhenIdle(false);   // <-- sleepy end device

  digitalWrite(LED_PIN, LED_ON);   delay(100);
  digitalWrite(LED_PIN, LED_OFF);  delay(100);
  digitalWrite(LED_PIN, LED_ON);   delay(100);
  digitalWrite(LED_PIN, LED_OFF);

  if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
    // Do NOT ESP.restart() here: a software reset does not reliably re-init the C6's
    // 802.15.4 radio, so it would just fail again forever. A deep-sleep wake power-
    // cycles the radio. If this persists, the pairing is stale -> re-pair.
    Serial.println("Zigbee start/join failed -> deep sleep, retry on wake");
    Serial.flush();
    // Also allow the reset button to wake us out of the retry cycle (held 5s -> reset).
    esp_deep_sleep_enable_gpio_wakeup(1ULL << BOOT_BUTTON, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_sleep_enable_timer_wakeup((uint64_t)ZB_JOIN_RETRY_S * 1000000ULL);
    esp_deep_sleep_start();
  }
  // "Awake but not joined": poll the button so a 5s hold here also factory-resets.
  Serial.print("Connecting to network");
  while (!Zigbee.connected()) {
    if (resetButtonHeld()) doFactoryReset();
    Serial.print("."); delay(100);
  }
  Serial.println(" connected");

  // First pairing (a real power-on, not a deep-sleep wake): stay awake for the interview.
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  if (wc != ESP_SLEEP_WAKEUP_GPIO && wc != ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Fresh boot -> staying awake ~10s for the Zigbee interview");
    uint32_t t0 = millis();
    while (millis() - t0 < 10000UL) {
      if (resetButtonHeld()) doFactoryReset();
      delay(100);
    }
    Serial.println("Interview window done");
  }

  // --- Battery on every wake (both as unsolicited reports) ---
  zbOccupancy.setBatteryPercentage(pct);
  zbOccupancy.reportBatteryPercentage();
  zbBattV.setAnalogInput(mv / 1000.0f);   // volts (e.g. 3.894)
  zbBattV.reportAnalogInput();

  // --- Publish the current occupancy-hold setpoint so z2m shows the real value ---
  zbConfig.setAnalogOutput((float)occupancyHoldS);
  zbConfig.reportAnalogOutput();

  // Printed AFTER connect so it survives the post-wake USB-CDC re-enumeration.
  Serial.printf("Battery: %u mV  ->  %u%%  (%.3f V)   Hold: %u s\n",
                mv, pct, mv / 1000.0f, occupancyHoldS);

  // Stuck-line safety must not cut a legitimately long hold short: allow at least the
  // configured hold (plus margin), but never less than MAX_AWAKE_S.
  uint32_t maxAwakeS = (occupancyHoldS + 30 > MAX_AWAKE_S) ? (occupancyHoldS + 30) : MAX_AWAKE_S;

  // A GPIO wake *is* a motion event by definition: the buffered PIR line pulled D1 LOW to
  // wake us. Report occupied on that fact alone, even if the (short) AM312 pulse already
  // ended during boot+connect -- otherwise a brief, distant movement (~3-5 m) would be lost.
  // EXCEPT when the reset button was the pin that pulled a GPIO LOW (btnLowAtWake): that is a
  // button interaction, not motion, so it must not raise a false occupancy report.
  bool motionWake = (wc == ESP_SLEEP_WAKEUP_GPIO) && !btnLowAtWake;

  // --- Occupancy FIRST, reported immediately. Nothing (no config window, no extra delay)
  //     is allowed to run before this, so even the shortest movement is captured. ---
  if (motionWake || motionPresent()) {
    Serial.println(motionWake ? "Motion wake -> occupied" : "Motion present -> occupied");
    digitalWrite(LED_PIN, LED_ON);
    zbOccupancy.setOccupancy(true);
    zbOccupancy.report();

    uint32_t occupiedStart = millis();
    uint32_t lastMotion    = millis();
    uint32_t lastDbg       = 0;
    // Holds occupancyHoldS from the last seen motion. This loop also keeps the radio awake
    // and polling, so it doubles as the receive window for a queued config write on motion wakes.
    while (millis() - lastMotion < occupancyHoldS * 1000UL) {
      if (motionPresent()) lastMotion = millis();
      if (resetButtonHeld()) doFactoryReset();   // reset works even while occupied

      // Safety: never stay awake forever if the buffered line is stuck asserted.
      if (millis() - occupiedStart > maxAwakeS * 1000UL) {
        Serial.println("MAX_AWAKE reached -> forcing clear (is D1 stuck LOW?)");
        break;
      }
      // Throttled: shows the raw buffered PIR level. HIGH = idle, LOW = motion.
      if (millis() - lastDbg > 2000UL) {
        lastDbg = millis();
        digitalWrite(LED_PIN, motionPresent() ? LED_ON : LED_OFF);
        Serial.printf("  D1=%d (%s)\n", digitalRead(PIR_PIN),
                      motionPresent() ? "motion" : "idle");
      }
      delay(50);
    }

    Serial.println("No motion -> cleared");
    digitalWrite(LED_PIN, LED_OFF);
    zbOccupancy.setOccupancy(false);
    zbOccupancy.report();
  } else {
    digitalWrite(LED_PIN, LED_OFF);
    zbOccupancy.setOccupancy(false);
    zbOccupancy.report();
  }

  // --- Config-receive window, AFTER occupancy is handled so it can never delay a motion
  //     report. Mainly serves heartbeat (no-motion) wakes: gives the stack time to poll the
  //     parent and deliver any queued genAnalogOutput write from z2m. On motion wakes the
  //     hold loop above already provided this time. ---
  uint32_t tCfg = millis();
  while (millis() - tCfg < (uint32_t)CONFIG_LISTEN_S * 1000UL) {
    if (resetButtonHeld()) doFactoryReset();
    delay(50);
  }
  applyPendingHold();

  delay(200);          // let the last report leave the radio
  goToDeepSleep();
}

void loop() {
  // Unused: setup() always ends in deep sleep, which restarts the sketch on wake.
}