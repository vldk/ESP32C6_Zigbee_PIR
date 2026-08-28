# ESP32-C6 Zigbee PIR presence sensor (ESP-IDF)

Native ESP-IDF port of `docs/esp32c6_AM312_pullup_v3_2.ino`, with the deep-sleep
cycle replaced by the esp-zigbee sleep feature (automatic light sleep).

- Board: Seeed XIAO ESP32-C6
- Stack: ESP-IDF 6.0.1 + `espressif/esp-zigbee-lib` 1.6.x (Zigbee End Device)
- Build: PlatformIO, `framework = espidf`

---

## Why the sleep mode changed

The Arduino sketch did all its work inside `setup()` and ended every wake in
`esp_deep_sleep_start()`. Deep sleep wipes RAM, so each motion event paid for a
full boot, radio init and parent rejoin before the occupancy report could leave
the device.

Light sleep keeps RAM, the radio state and the network join intact. The Zigbee
stack raises `ESP_ZB_COMMON_SIGNAL_CAN_SLEEP` whenever it is idle;
`esp_zb_sleep_now()` releases its power-management lock, and the IDF tickless
idle takes the SoC down until the next parent poll, GPIO edge or timer alarm.

| | Deep sleep (sketch) | Light sleep (this port) |
|---|---|---|
| Motion report latency | boot + rejoin | milliseconds |
| RAM across sleep | lost | retained |
| Motion vs. button wake | ambiguous, disambiguated by re-reading GPIO7 | distinct interrupts |
| z2m config write | needed an explicit `CONFIG_LISTEN_S` awake window | arrives on the next ordinary parent poll |
| Occupancy hold | busy-wait loop with `delay(50)` | `esp_timer`, asleep in between |

---

## Runtime shape

Every module is a producer; `app_task` in `main.c` is the only consumer, so all
application state lives on one thread and needs no locking.

```
   GPIO1 (PIR) ──┐  level ISR
   GPIO7 (btn) ──┤  + light-sleep wake
                 │
        board_io ├──────────────┐
                 │              │
   esp_timer ────┤ hold         ├──►  s_events queue  ──►  app_task
   alarms        │ heartbeat    │                            │
                 │ reset-hold   │                            │
                 │              │                            ▼
        zb_sensor├──────────────┘                    set_occupancy()
        (stack task)                                 report_battery()
             ▲                                       settings_set_hold_s()
             └───────────────────────────────────────────────┘
                        esp_zb_lock_acquire()
```

| File | Responsibility |
|---|---|
| `src/app_config.h` | every pin and tunable, single source of truth |
| `src/app_events.h` | the one message type crossing thread boundaries |
| `src/board_io.c` | PIR / button / LED, GPIO light-sleep wake |
| `src/battery.c` | ADC oneshot + curve-fitting calibration -> mV, % |
| `src/settings.c` | NVS store for the occupancy hold setpoint |
| `src/zb_sensor.c` | endpoints, clusters, reports, signal handler, sleep hook |
| `src/main.c` | wiring and the occupancy state machine |

---

## The one subtle part: level-triggered GPIO

The GPIO peripheral is clocked from APB, which stops during light sleep, so an
**edge** that happens while the chip sleeps is lost. Only a **level** survives:
`gpio_wakeup_enable()` wakes the chip while the pin sits at the configured
level, and that level is still there when the ISR finally runs.

So `board_io.c` arms each input for the level it does *not* currently have —
for both the wake source and the interrupt — and flips the watch after every
transition:

```
idle (HIGH) ──arm LOW──►  motion asserted (LOW) ──arm HIGH──►  released ──►  idle
```

The ISR masks the pin (a level interrupt would re-enter forever) and the
application calls `board_io_rearm()` once it has acted. If the line flips back
between the two, the newly armed level already matches and the interrupt fires
immediately — one spurious wake, then self-corrected, never a lost edge.

Two related sdkconfig points:

- `CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP=n` — powering the HP
  peripherals down takes the GPIO block with them and makes
  `esp_sleep_enable_gpio_wakeup()` unavailable.
- `CONFIG_PM_SLP_DISABLE_GPIO` is on by default and parks every pad during
  sleep; `gpio_sleep_sel_dis()` is the documented opt-out, applied to the PIR,
  the button and the LED.

---

## Zigbee model

Unchanged from the sketch, so an existing zigbee2mqtt converter keeps working.
All reports are sent **unsolicited**: a sleepy end device never completes the
bind + configureReporting handshake z2m normally does, because it times out
while the device is asleep.

| EP | Cluster | Purpose |
|---|---|---|
| 10 | Occupancy Sensing | `occupancy` |
| 10 | Power Config | `batteryPercentageRemaining` (half-percent units) |
| 11 | Analog Input | battery volts — reportable, unlike the read-only Power-Config `BatteryVoltage` |
| 12 | Analog Output | occupancy hold setpoint in seconds, writable from z2m |

A write to `genAnalogOutput.presentValue` on EP 12 is clamped to
`OCCUPANCY_HOLD_MIN_S..OCCUPANCY_HOLD_MAX_S`, persisted to NVS only when it
actually changed, and echoed back so z2m displays the accepted value. It also
applies to an occupancy period already in progress.

The z2m external converter is `docs/esp32c6_AM312_pullup.mjs`. It accepts both
`AM312_Presence_v2` (the Arduino sketch) and `AM312_Presence_v3` (this port),
since the Zigbee model is identical between them - only the model identifier
string changed.

---

## Hardware

```
  AM312 OUT --[100k]--> Base (2N3904 / BC547 / BC337)
                        Emitter --> GND
                        Collector --> D1 / GPIO1
  D1 / GPIO1 --[1M]--> 3V3          (idle pull-up)

  No motion : AM312 OUT = 0V   -> NPN off -> D1 = HIGH
  Motion    : AM312 OUT = 1.8V -> NPN on  -> D1 = LOW
```

- Battery: LiPo on the BAT pads, `2x1M` divider midpoint -> D0 / GPIO0.
- Factory reset: external momentary switch from GPIO7 (D7) to GND, held 5 s.
  GPIO7 has no on-board button — the physical BOOT button is the GPIO9 strap
  pin and is unusable here.

---

## Build and flash

```bash
pio run                  # build
pio run -t upload        # flash
pio device monitor       # logs (115200, USB-Serial-JTAG)
pio run -t menuconfig    # stack / PM options
```

`sdkconfig.defaults` is only consulted when the per-environment `sdkconfig` is
generated. After editing it, delete `sdkconfig.seeed_xiao_esp32c6` and rebuild.

`board_build.partitions` in `platformio.ini` has to name the same file as
`CONFIG_PARTITION_TABLE_CUSTOM_FILENAME`: PlatformIO generates the partition
table itself and ignores the sdkconfig value. Without it the build silently
falls back to the single-app table and the stack loses `zb_storage`/`zb_fct`.

### Battery measurements

Logging over USB-Serial-JTAG keeps that peripheral alive and dominates the
sleep current. For a real consumption figure, set
`CONFIG_LOG_DEFAULT_LEVEL_NONE=y` and run the board off the battery.
