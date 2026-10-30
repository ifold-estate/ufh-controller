# UFH Controller — Project Context and Handoff

This file carries the decisions and background from the first working session, so a new session
(human or Claude) can pick up the project. Keep it updated as decisions change.

_Last updated: 30 Oct 2026. Ethernet/PoE dropped: the installed controller runs on Wi-Fi with local power._

---

## 1. Goal

A controller for an underfloor heating (UFH) manifold and mixer, integrated with a local
Home Assistant (HA) server. The heating logic must run **on the controller itself**, so heating keeps
working if HA or the network is down. HA is used for monitoring, setpoints and overrides.

The owner (Jon) isn't familiar with the ESP32 environment, so **Claude writes and maintains the code**;
Jon does the wiring, flashing and testing.

## 2. Hardware

| Item | Details |
|---|---|
| Controller | **Waveshare ESP32-S3-POE-ETH-8DI-8RO**: ESP32-S3-WROOM-1U-N16R8 (16 MB flash, 8 MB octal PSRAM), W5500 Ethernet, PoE (802.3af) sub-module, 8 relays (10 A 250 Vac), 8 opto-isolated digital inputs, isolated RS485, PCF85063 RTC, TF slot, buzzer, WS2812 LED, 28-pin header. No CAN on this model. Docs: https://www.waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO (the PoE version uses the same baseboard). **Not** the 8DI-8DO board, which has transistor outputs. |
| Zone valves | 24 Vac solenoids on the manifold, switched by the relays |
| Thermostats | NEST v1 and v2, running on 24 Vac. Their call-for-heat output must reach the DIs (interface not designed yet; see §7). |
| Pump | 240 Vac, switched by a relay |
| Boiler call for heat (CFH) | 240 Vac signal, switched by a relay |
| Mixer | Mechanical mixing valve, turned by a **Waveshare SC15 bus servo** (Feetech SCS series, TTL half-duplex serial, 4.8–8.4 V, 15 kg·cm at 6 V, 0–1023 position over about 200°, feedback for position, speed, load, voltage and temperature). Load feedback will be used to find the valve's mechanical end stops. |
| Servo interface | **Waveshare Bus Servo Adapter A**. Jumper on A (UART mode); B is USB mode. Its input supply passes straight to the servo (SC15: 4.8–8.4 V; the bench uses 5.2 V). Its UART logic is 3.3 V (TXD input 5 V tolerant), so it connects straight to the ESP32. |
| Temperature sensors | Several DS18B20s (Jon already has them) |
| Display | **Waveshare 2.4" LCD Module**: ILI9341, 240×320, 4-wire SPI, no touch, pins VCC GND DIN CLK CS DC RST BL. Its logic level follows VCC, so it must be powered from 3V3. https://www.waveshare.com/wiki/2.4inch_LCD_Module |
| Parts on hand | Logic-level converter board, rectifier, PSU options |

## 3. Key decisions (and why)

1. **Firmware framework: ESPHome** (ESP-IDF framework). It integrates natively with HA through
   automatic discovery and the API, has on-device automations, thermostat and PID control, supports
   the W5500, TCA9554, PCF85063 and DS18B20, and its YAML and C++ are text Claude can maintain.
   Arduino/PlatformIO with MQTT and raw ESP-IDF were rejected as more code for the same result.
2. **Network is Wi-Fi; updates go over Wi-Fi (OTA)** after a first flash over USB-C. A network
   cable can't be pulled to the cabinet (30 Oct 2026), so the installed controller runs
   `ufh-controller-wifi.yaml` and Ethernet/PoE are unused. Fit the antenna outside or at the edge
   of a metal enclosure and check the "Wi-Fi signal" entity (aim for better than about −70 dBm).
   Heating control must not depend on Wi-Fi or HA being up, so the API's reboot-on-HA-loss is off
   (`reboot_timeout: 0s`); Wi-Fi keeps its default 15 min reboot to recover a stuck connection.
   The fallback "UFH Controller Setup" hotspot is WPA2-protected with `ap_password` (secrets.yaml
   locally, the `AP_PASSWORD` Actions secret in CI). Keep USB-C reachable in the enclosure
   for recovery.
3. **Servo on a plain UART from the header (Tx/Rx, GPIO43/44), not USB.** The ESP32-S3 can be a USB host, but
   that would take over the only USB port (needed for flashing, logs and recovery), USB-C isn't wired
   to supply VBUS, and ESPHome's USB-serial support is newer and less proven.
4. **The DI terminals can't be used for serial.** They are opto-isolated (5–36 V), input-only and slow.
5. **Don't power the board from 24 Vac.** The input is rated 7–36 V **DC**. 24 Vac peaks around 34 V
   and reaches roughly 37–40 V on a lightly loaded transformer, so it would overstress the input's TVS
   diode. A bridge rectifier doesn't help, since it gives the same peak voltage as DC. With PoE ruled
   out, **use a 24 Vdc (or 12 Vdc) DIN-rail PSU** from the cabinet's mains (e.g. Mean Well
   HDR-15-24).
6. **Logs go to USB Serial/JTAG**, so UART0 (header Tx/Rx, GPIO43/44) is free for the servo. The
   boot ROM prints start-up text on GPIO43 at 115200, but it has no 0xFF bytes, so it can't form a
   valid servo packet. (The servo was first planned for GPIO1/2; it moved because the header's Tx/Rx
   pins sit next to 3V3 and G, so one 4-pin connector carries everything.)
7. **The LCD uses the TF-card pins (47/48)**, so the TF card is not used. CS and BL are tied to fixed
   levels, so there's no backlight control. RST must be driven from GPIO0: with CS tied low and
   RST tied high, the panel stayed white, since it can't resync after stray clock edges at power-up.
   GPIO0 (BOOT) is high at power-on, so sharing it with the LCD reset is safe. The display runs at
   10 MHz in SPI mode 0 (set while debugging; keep them unless there's a reason to change).
8. **Servo link: 115200 baud, wired direct, no level shifter.** The adapter's schematic shows 3.3 V
   logic (SN74LVC1G125/126 on an AMS1117-3.3; RXD drives 0–3.3 V, TXD is 5 V tolerant). A TXS0108E
   shifter (HW-221) was tried first and was unreliable: its 5 V side needs ~3.4 V for a high, so the
   adapter's 3.3 V replies were marginal. The servo was set to 115200 with `tools/scs_tool.py`. A replacement servo needs the same change. Wiring is
   straight through, as Waveshare says: ESP Tx → adapter TX, ESP Rx → adapter RX (confirmed).

## 4. Pin map

Fixed by the board:

| Function | GPIO |
|---|---|
| DI1–DI8 | 4, 5, 6, 7, 8, 9, 10, 11 (active low, inverted in config) |
| Relays 1–8 | TCA9554 @ 0x20, expander pins 0–7 (EXIO1–8) |
| I2C (expander, RTC) | SDA 42, SCL 41 |
| W5500 (SPI2) | SCLK 15, MOSI 13, MISO 14, CS 16, INT 12 |
| RS485 | TX 17, RX 18 (automatic direction control) |
| TF card | 45, 47, 48 |
| WS2812 LED | 38 |
| Buzzer | 46 (strapping pin) |
| BOOT | 0 |

Header (2×14), read from Waveshare's board photo:

```
Top:    5V   5V   G    G    3V3  G    SCL  SDA  3V3  G    21   45   47   48
Bottom: Vbus G    DP   DN   3V3  G    Tx   Rx   3V3  G    0    1    2    3
```

Project allocation:

| Pin | Use |
|---|---|
| GPIO21 | DS18B20 one-wire bus, 4.7 kΩ pull-up to 3V3 |
| GPIO48 | LCD CLK (SPI3) |
| GPIO47 | LCD DIN (SPI3) |
| GPIO3 | LCD DC (strapping pin, fine as an output) |
| GPIO0 | LCD RST (BOOT pin; pressing BOOT also resets the LCD) |
| LCD CS → G, BL → 3V3, VCC → 3V3 | |
| GPIO43 (Tx) | Servo UART TX → adapter TX (direct) |
| GPIO44 (Rx) | Servo UART RX ← adapter RX (direct) |
| GPIO45 | Avoid (strapping pin for flash voltage) |
| Spare | GPIO1 and GPIO2. I2C (41/42) can take more devices. |

## 5. Code status (v0.1)

Repository: `ifold-estate/ufh-controller`

- `ufh-controller.yaml` sets up:
  - Ethernet (unused in the installation), the API (encrypted), OTA (encrypted with the API key), and the web server with auth
  - I2C, the TCA9554 relays (all `ALWAYS_OFF` at boot), DI1–8 with a 50 ms filter
  - The PCF85063 RTC (set from HA time), status, uptime, buzzer (RTTTL) and RGB LED
  - One DS18B20 placeholder (add addresses from the boot log)
  - The servo entities, and the LCD heat-flow screen (`ufh_screen.h`, from Jon's sketch, agreed
    30 Oct 2026): boiler loop, mixer and floor loop with pipes coloured by probe temperature
    (10 °C blue → sky blue → amber → 35 °C orange → 60 °C red, avoiding a purple midpoint); flame
    and pumps follow relays 8 and 7; zone dots follow relays 1–6; room names follow the thermostat
    DIs 1–4 (blue = calling for heat); bubbles move clockwise round a loop while its pump runs (redraw
    every 200 ms, otherwise every 2 s); mode pill and status word at the bottom. Preview it on a PC
    with `tools/screen_preview/render.py OUT_DIR`. Mode is fixed at MANUAL and status is derived
    from the relays until the heating logic sets them. Not yet seen on the real LCD; if the
    animation is jerky, try the SPI at 40 MHz (a full redraw takes ~120 ms at 10 MHz).
- `ufh-controller-wifi.yaml` is the production build. It includes the main config, swaps Ethernet
  for Wi-Fi (ESPHome can't run both at once), adds a "Wi-Fi signal" diagnostic sensor, and has no
  Wi-Fi password built in. The password is set
  after flashing, over USB (Improv on web.esphome.io) or through the "UFH Controller Setup" hotspot.
  Wi-Fi needs an antenna on the board's SMA socket, since the WROOM-1U has no PCB antenna. CI builds
  both variants.
- `components/scs_servo/` is a custom external component implementing the Feetech SCS protocol
  (big-endian registers, echo-tolerant half-duplex).
  - Registers: torque 40, goal position 42, present position 56, speed 58, load 60 (bit 10 is the
    sign, 0–1000), voltage 62 (0.1 V), temperature 63.
  - It is a PollingComponent with sensors, a `number` (goal position) and a `switch` (torque).
  - Lambda API: `set_goal_position()`, `set_torque()`, `ping()`, `is_online()`, `last_position()`,
    `last_load_percent()`.
- Fonts: DejaVu Sans, bundled in `fonts/`.
- `secrets.yaml` is not committed. `secrets.example.yaml` holds `api_encryption_key` and
  `web_password`.
- `.github/workflows/build.yml` compiles the firmware on every push and uploads the `.bin` files.
  It builds `secrets.yaml` from the repo's Actions secrets `API_ENCRYPTION_KEY` and `WEB_PASSWORD`.
  If those are missing (for example, a PR from a fork), it falls back to throwaway values and names
  the artifact `ufh-controller-firmware-untrusted`. Don't flash that one.

Verification so far:

- `esphome config` passes on ESPHome 2026.10.1.
- The servo C++ was syntax-checked on its own.
- CI compiles both the Ethernet and Wi-Fi variants.
- Bench test so far (Wi-Fi variant, flashed over USB then OTA): Wi-Fi, the encrypted API and OTA
  work, the TCA9554 (0x20) and RTC (0x51) respond on I2C, and the LCD test page displays.
  Relays 1–8 click from the web page, with their LEDs and the LCD following. The status LED is RGB
  order, not GRB. The servo answers at 115200 baud
  (position, load, voltage and temperature read correctly; about 300 polls at 50 ms with no errors).
  Movement checked over the native API (no load, no end stops): every move from 300 to 700 lands
  within 2–4 steps of the target, a 400-step move finishes within one 2 s poll, and the load reads
  about ±6–8 % while holding and 0 % with torque off. Writing a target with torque off still moves
  the servo, because the servo re-enables torque itself, so the torque switch can show OFF while
  the servo is holding. The built-in web server stopped answering HTTP once during testing, while
  the API kept working.
- DS18B20s: three probes on GPIO21, named Boiler flow, Return and Mixed flow by address. Identified
  by warming each one and labelled. They read room temperature (19–21 °C). The LCD shows all three.
- Servo link through the TXS0108E was unreliable (bursts of flipped bits, even with the servo idle),
  because the adapter's 3.3 V high is below the shifter's ~3.4 V threshold. Wired direct, it's
  clean (15/15 reads, no errors). The torque switch now reads back the servo's torque register, so
  it shows ON when a goal position re-enables torque.
- HA (2026.10.2 at 192.168.1.194) discovered and adopted the controller; new entities appear in HA
  after each firmware update without re-adding it. Removed entities (e.g. the old "Temperature
  probe 1") linger in HA as unavailable and have to be deleted there.
- Not confirmed yet: DIs. Ethernet is untested and no longer planned. Don't flash
  `ufh-controller.yaml` (the Ethernet build) to the installed board: it would drop off Wi-Fi with
  no network, and recovery needs USB-C.

## 6. Environment notes (for Claude sessions)

- In the cloud workspace, install ESPHome with `python3 -m venv .venv && .venv/bin/pip install esphome`.
  System pip is blocked.
- The cloud workspace **can't download the Espressif toolchain or Google Fonts** (proxy 403), so
  `esphome compile` fails there. Validate with `esphome config` and rely on CI for full builds.
- Git push needs the repo attached to the session. Personal access tokens are overridden by the
  session's GitHub proxy.
- Jon's Mac can be linked to the session, and the built-in browser works there (waveshare.com
  allowed).

## 7. Open questions and next steps

**Questions for Jon:**

- [ ] How is HA installed: HA OS/Supervised (ESPHome add-on available) or Container/Core? (HA 2026.10.2 runs at 192.168.1.194:8123; install type still unknown.)
- [ ] How many zones are there, and which relay drives what? Proposed: relays 1–6 zones, 7 pump,
      8 boiler CFH.
- [x] What logic level does the Bus Servo Adapter A UART use? 3.3 V (TXD 5 V tolerant), so no level shifter.
- [ ] NEST → DI interface. The DIs are bi-directional opto inputs rated 5–36 V. A 24 Vac call-for-heat
      line peaks above 36 V, so it needs a design: a small 24 Vac relay per zone, or a resistor/opto
      interface. Also, NEST thermostats need a common wire or power-stealing, which affects how CFH
      is sensed.
- [ ] Which DS18B20 positions: flow (after the mixer) is essential, and return is recommended.
      Anything else, such as the boiler flow side?

**Next work:**

1. ~~Push v0.1, let CI compile it, and fix any build errors.~~ Done.
2. Bench test: flash over USB-C, check Wi-Fi, HA discovery, relays, DIs and the LCD. (Ethernet dropped.)
3. Servo bring-up: ping, read position and load, move it, then tune the response timeout and baud.
4. Limit-finding routine: drive slowly to each end until the load rises, store the end-stop
   positions in flash, and map 0–100 % mixer opening onto that range. Use the goal-speed register
   (46) to creep during calibration, and keep normal moves a little short of the stops.
   **Polling design (agreed):** poll the servo every 2 s while idle and about every 100 ms while it
   moves or calibrates. The fast loop stays on the device. HA gets only filtered values on a slower
   schedule (e.g. position change > 2 steps, load change > 5 %, plus a periodic heartbeat), so the
   HA database doesn't fill with repeated readings. The servo is fast (under 2 s for 400 steps);
   that's fine for control, since the thermal loop is much slower, and speed can be capped later.
**Temperature sensors (planned): three DS18B20s at the mixer's ports.** Boiler flow (hot in),
return (cold in, from the manifold) and mixed flow (out to the manifold). Mixed flow is the PID's
control input and drives the over-temperature cut-out. Boiler flow and return show how much heat the
floors take and whether the boiler is working.

**Plant timing (from Jon, 30 Oct 2026):** the boiler has a high thermal mass and is some distance
away, and its CFH has a hysteresis of 1–2 minutes, so it takes about 5 minutes from a thermostat
call to hot water at the mixer. The zone actuators take about 2 minutes to open. Design consequences:
- Boiler fault: judge on boiler flow *rising* after a grace period (e.g. 15 min), not on a fixed
  temperature soon after the call.
- PID: hold it while boiler flow is below setpoint + margin, to avoid integral windup, and reset it
  when it starts. Pre-position the valve from all three temperatures, roughly
  (setpoint − return) / (boiler flow − return), and let the PID trim, to limit overshoot when hot
  water arrives.
- Sequencing: zone valves open → wait for the actuator opening time → pump on → boiler CFH. When
  demand ends: drop CFH, pump overrun to carry away stored heat.
- Over-temperature cut-out stays immediate.
- All timings (actuator opening, pump overrun, boiler grace period) are adjustable from HA.

5. Heating logic on the device:
   - zone demand → solenoids
   - pump on when any zone is open (with overrun)
   - boiler CFH only while the pump runs
   - PID on flow temperature → mixer position
6. Fail-safes:
   - flow over-temperature cut-out (close the mixer, drop CFH)
   - safe behaviour on sensor loss or servo offline
   - watchdog defaults while HA is unreachable
7. LCD: have the heating logic drive the mode (AUTO / OVERRIDE / MANUAL) and status (Warming, Holding, Waiting…).
