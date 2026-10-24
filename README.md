# rexaUi - A Joystick-Driven Mini OS for the ESP32-S3 LCD

**rexaUi** is a complete, self-contained user interface and mini "launcher OS" for the
**Waveshare ESP32-S3-LCD-1.9** board, controlled with a **KY-023 analog joystick**.
It is written in C on top of ESP-IDF (built with PlatformIO) and draws everything with its own
small graphics engine: a smooth carousel home screen, widgets, a game, Wi-Fi/Bluetooth tools,
a JPG/GIF gallery, a Lua app runtime, battery/power management and RGB LED control.

> **A joystick is required.** rexaUi has no touch support and no other input method. Every screen
> is navigated with a KY-023 (or compatible) 2-axis analog joystick with a push button.
> Without it you cannot move through the menus. See [Wiring](#wiring).

---

## Table of contents

- [Features](#features)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Building and flashing](#building-and-flashing)
- [Controls](#controls)
- [Screens and modes](#screens-and-modes)
- [Settings](#settings)
- [SD card](#sd-card)
- [Lua apps](#lua-apps)
- [Power saving and battery estimate](#power-saving-and-battery-estimate)
- [Project layout](#project-layout)
- [Known limitations](#known-limitations)
- [License](#license)

---

## Features

- **Smooth landscape UI (320x170)** with a gradient carousel home screen and animated transitions.
- **Home widgets that work without opening them:** the clock card shows the time, the weather card
  shows current conditions and the next hours.
- **Clock** with a large digital display, NTP time sync over Wi-Fi and 70+ world cities with
  correct time zones and daylight-saving rules.
- **Weather** (Open-Meteo, no API key): type any city on the on-screen keyboard, the device finds it
  and shows temperature, feels-like, humidity, wind and a 6-hour forecast on a full screen.
- **Snake game.**
- **Wi-Fi manager:** scan, join open and password-protected networks with an on-screen keyboard
  (the password is visible while typing), automatic reconnect, signal bars in the status bar.
- **Bluetooth LE scanner** with basic GATT connect.
- **SD gallery:** JPG/JPEG and GIF files directly from the card root, correct colors and EXIF
  orientation, dithered output.
- **SD apps:** run `.lua` programs (and tiny `.rxa` manifests) from the card.
- **Sleep mode:** after a timeout the screen shows only a low-power digital clock; everything
  except what keeps the clock accurate (Wi-Fi/NTP) is shut down.
- **Power saver** with a battery-life estimate refreshed every 10 seconds.
- **RGB LED control** for the two on-board WS2812 LEDs: modes, colors, speed and brightness.
- **Screen flip and joystick flip** switches for mounting the board in either orientation.
- English and Turkish interface, persistent settings (NVS), default brightness 40%.

## Hardware

| Part | Notes |
| --- | --- |
| **Waveshare ESP32-S3-LCD-1.9** | ESP32-S3, 170x320 ST7789 LCD, microSD slot, 2 WS2812 LEDs, battery connector. Use the **non-touch** version. |
| **KY-023 joystick module** | **Required.** Two analog axes (VRx, VRy) and a push button (SW). |
| microSD card | Optional. FAT32. Needed for the gallery and SD apps. |
| Li-ion/LiPo battery | Optional. The battery-life estimate assumes a capacity you choose in the menu (default 600 mAh). |
| USB-C cable | For flashing and power. |

The firmware assumes 16 MB flash (see [boards/rexa-esp32-s3.json](boards/rexa-esp32-s3.json) and
[rexa_single_app.csv](rexa_single_app.csv)).

## Wiring

Everything on the board itself (LCD, SD slot, LEDs, battery sense) is already wired by Waveshare;
the table lists the GPIOs so you know which pins are in use. **Only the joystick has to be wired
by you.**

### Joystick (KY-023) - you wire this

| KY-023 pin | Connect to | ESP32-S3 GPIO | Function |
| --- | --- | ---: | --- |
| `GND` | GND | - | Ground |
| `+5V` (VCC) | **3V3** | - | Power the module from **3.3 V**, not 5 V |
| `VRx` | GPIO1 | **1** | X axis, analog (ADC1_CH0) |
| `VRy` | GPIO2 | **2** | Y axis, analog (ADC1_CH1) |
| `SW` | GPIO3 | **3** | Push button, active low (internal pull-up) |

Notes:

- **Power the joystick from 3.3 V.** The analog outputs go straight to ESP32-S3 ADC pins, which
  are not 5 V tolerant. Powering the module from 5 V can damage the chip.
- VRx and VRy must go to ADC1-capable pins; GPIO1 and GPIO2 are ADC1 channels 0 and 1.
- SW needs no external resistor; the firmware enables the internal pull-up.
- If a direction feels reversed because of how you mounted the stick, use
  **Settings > Flip joystick** instead of rewiring.

### On-board connections (for reference)

| Function | Signal | ESP32-S3 GPIO |
| --- | --- | ---: |
| **LCD (ST7789, SPI)** | MOSI | 13 |
| | SCLK | 10 |
| | D/C | 11 |
| | CS | 12 |
| | RST | 9 |
| | Backlight (PWM) | 14 |
| **microSD (SDMMC, 1-bit)** | CLK | 41 |
| | CMD | 39 |
| | D0 | 40 |
| **WS2812 LEDs (2x)** | DIN | 15 |
| **Battery voltage sense** | ADC (via divider) | 4 (ADC1_CH3) |
| **QMI8658 IMU (I2C)** | SDA / SCL | 47 / 48 |

### Display details

- Panel: ST7789, 170x320 pixels, driven over SPI.
- The UI is always **landscape, 320x170**. The panel is rotated in software; use
  **Settings > Flip screen** to rotate it by 180 degrees.
- The top 20 pixels are a system status bar (time, Wi-Fi bars and name, Bluetooth, battery).
  Lua apps get the remaining 320x150 area.
- Backlight is PWM controlled; the default brightness is **40%**.

## Building and flashing

You need [PlatformIO](https://platformio.org/) (VS Code extension or CLI). ESP-IDF and the
Lua component are downloaded automatically on the first build.

```bash
git clone <this repository>
cd <repository folder>
pio run                                   # build
pio run -t upload --upload-port COM11     # flash (use your own serial port)
pio device monitor                        # optional serial log
```

- `platformio.ini` contains `upload_port = COM11`, which is just the author's port. Change it to
  your own (for example `COM5` or `/dev/ttyACM0`) or pass `--upload-port`.
- If the board is not detected, hold **BOOT**, tap **RESET**, release BOOT, then flash again.
- Default configuration lives in `sdkconfig.defaults` (16 MB flash, Bluetooth LE, power management,
  mbedTLS certificate bundle). The generated `sdkconfig` is not committed.

## Controls

| Input | Action |
| --- | --- |
| Joystick left / right | Move through the home carousel; left goes back in most screens |
| Joystick up / down | Move through lists and settings, change values |
| **Button (SW)** | Open / select / toggle |
| Right (in lists) | Rescan (Wi-Fi, Bluetooth, SD apps, gallery) |

In keyboards, the joystick moves the highlighted key and the button types it. Exact directions can
be inverted with **Settings > Flip joystick** (and the screen with **Flip screen**), so the controls
feel natural however you mount the board.

## Screens and modes

### Home carousel
The home screen is a gradient carousel with the clock and weather widgets live on the cards.
Entries: Clock, Weather, Game, Bluetooth, Language, City / Time, SD Apps, Gallery, LED, Power,
Settings.

### Clock
Large digital clock with date. Pick a city in **City / Time** to set the time zone; the time
syncs through NTP (`pool.ntp.org`) when Wi-Fi is connected. Before the first sync the clock starts
from the firmware build time and keeps running while the device is on. There is no battery-backed
RTC on this board.

### Weather
Full-screen weather for a location you choose:

1. Open **Weather** and press the button.
2. Type a city name on the keyboard (English letters; for example `ISTANBUL`, `PARIS`, `TOKYO`).
3. Select **OK**. The device looks the place up through the Open-Meteo geocoding service and shows
   the result with its country code (for example `PARIS, FR`).

The screen shows temperature, condition icon, feels-like, humidity, wind and a 6-hour forecast. The
chosen location is stored and reused by the home weather widget. Weather needs an active Wi-Fi
connection with internet access.

### Game
A Snake game. Steer with the joystick, button to restart after game over, left to exit.

### Wi-Fi
Scan, select a network and join. Locked networks open a keyboard; the password is shown as you type
(8-63 characters). Keys: `BS` backspace, `CAP` upper/lower case, `SYM` symbols, `SP` space,
`CLR` clear, `OK` connect, `BK` back. The connected network name appears in Settings and the signal
bars are always visible in the status bar. Wi-Fi reconnects automatically.

### Bluetooth
Scans for BLE devices. Up/down selects, right rescans, the button starts a basic GATT connection.
No device-specific pairing or data transfer is implemented.

### Gallery
Shows **JPG/JPEG** and **GIF** files from the card root (see [SD card](#sd-card)). Colors are
corrected and EXIF orientation is respected. In the file list, up/down selects a file, the button
opens it and right rescans the card. While viewing, up/down switches to the previous/next file and
left or the button returns to the list. Videos are not supported.

### LED
Controls the two on-board RGB LEDs.

- **Mode:** Off, Solid, Breathe, Blink, Rainbow, Alternate
- **Color:** Red, Orange, Yellow, Green, Cyan, Blue, Purple, White
- **Speed:** 5 levels
- **Brightness:** 10-100% in steps of 10

A live preview circle shows the current color. LEDs are forced off when Power Saver is active and
during sleep.

### Power
Power Saver switch, battery capacity selection and the estimated remaining time. See
[Power saving](#power-saving-and-battery-estimate).

### Sleep mode
After the configured idle time, the screen shows **only a digital clock**. The CPU is slowed, the
backlight is dimmed and everything that is not needed to keep the clock accurate (only Wi-Fi/NTP
stay) is switched off. Any joystick movement or button press wakes the device and restores your
brightness.

## Settings

| Item | Description |
| --- | --- |
| Light | Screen brightness in 5% steps (default 40%) |
| System | Battery voltage, estimated time left, power source, CPU speed, free RAM, uptime |
| Wi-Fi | Scan and join networks; shows the connected network name |
| Sleep timer | 0 (off) or 1-120 minutes |
| Wi-Fi switch | Turn the radio on or off |
| Bluetooth switch | Turn Bluetooth on or off |
| Flip screen | Rotate the display 180 degrees |
| Flip joystick | Invert the joystick directions |
| Power saver | Enable the low-power profile |
| LED | Open the LED menu |

All settings are saved in NVS and survive restarts.

## SD card

1. Format the card as **FAT32** (this erases the card).
2. For **gallery**: copy `.jpg`, `.jpeg` or `.gif` files straight into the **root** of the card. They
   do not need to be placed in a folder or converted.
3. For **apps**: create `REXA/APPS` on the card and copy your `.lua` (or `.rxa`) files into it.

Notes:

- The card is opened read-only. Removing the card clears the Lua runtime and the file list from RAM.
- Progressive JPEG is not supported; use baseline JPEG.
- The SD interface uses a single data line (1-bit SDMMC) on GPIO39/40/41.

## Lua apps

Open **SD Apps**, pick an app with up/down and press the button to run it; press the button again to
close it. If the list is empty, push right to rescan. Up to eight apps are listed.

Lua 5.5 scripts are limited to 16 KB, 128 KB of RAM and a per-frame instruction budget. File, OS and
module-loading libraries are not available, and network access is limited to the fixed Open-Meteo
HTTPS endpoint.

Optional entry points:

```lua
function update(input, dt) end   -- called every frame
function draw() end              -- called every frame
```

The `rexa` API:

| Function | Description |
| --- | --- |
| `rexa.clear(rgb?)` | Clear the app area. Colors are `0xRRGGBB`. |
| `rexa.text(x, y, text, rgb?, scale?)` | Draw text, scale 1-3 |
| `rexa.fill_rect(x, y, w, h, rgb?)` | Filled rectangle |
| `rexa.circle(x, y, r, rgb?)` | Circle |
| `rexa.input()` | `{direction = "up"/"down"/"left"/"right"/"none", pressed = bool}` |
| `rexa.clock()` | `{year, month, day, hour, minute, second, unix}` or `nil` |
| `rexa.wifi()` | `{enabled, connected, state, ssid}` |
| `rexa.weather(lat, lon)` | Current weather table, or `nil, error` |
| `rexa.ble_scan()`, `rexa.ble_devices()`, `rexa.ble_connect(i)` | BLE scan, list and connect |
| `rexa.imu()` | QMI8658 acceleration (m/s2), gyro (deg/s), temperature, or `nil, error` |

Examples are in [examples/sd_apps/](examples/sd_apps): `clock.lua`, `weather.lua`,
`sensors_ble.lua` and `counter.rxa`. Copy them to `REXA/APPS/` on the card.

### `.rxa` manifests
A tiny text file (max 2 KB) that shows a ready-made screen instead of running code:

```ini
REXAUI/1
NAME=Counter
TYPE=counter
TEXT=Move left or right
COLOR=37DFDC
```

`TYPE` is `counter`, `clock` or `message`. `NAME` is up to 23 and `TEXT` up to 71 printable ASCII
characters; `COLOR` is a six-digit RGB hex value.

## Power saving and battery estimate

**Power saver** (Power menu or Settings):

- CPU lowered to 80 MHz
- Screen brightness capped at 20%
- LEDs off
- Wi-Fi modem power save
- Sleep after 1 minute of inactivity
- Slower frame rate

**Estimated time left** is recalculated every **10 seconds**. The board has no fuel gauge or current
sensor, so the estimate is **model based**: it combines the battery percentage (from the voltage on
GPIO4, smoothed) with a modeled current draw (CPU speed, Wi-Fi, Bluetooth, brightness, SD, LEDs) and
the capacity you set under **Power > Battery size** (default 600 mAh). Set the real capacity of your
battery for a meaningful value. While on USB, the voltage reading is not a reliable battery level, so
the value is only indicative.

## Project layout

```
src/main.c            Firmware: UI, drivers, Wi-Fi, BLE, gallery, Lua runtime, power, LEDs
src/world_cities.h    World city list (clock time zones)
src/CMakeLists.txt    ESP-IDF component definition
src/idf_component.yml Managed components (Lua)
boards/               PlatformIO board definition (16 MB flash)
rexa_single_app.csv   Partition table
sdkconfig.defaults    Default ESP-IDF configuration
examples/sd_apps/     Example Lua apps and an .rxa manifest
platformio.ini        PlatformIO project
```

## Known limitations

- A **joystick is mandatory**; there is no touch or button-only navigation.
- Weather, city search and NTP need Wi-Fi with internet access.
- City names must be typed with the on-screen character set (ASCII letters, digits, a few symbols).
- No video playback; progressive JPEG is not supported.
- Battery time is an estimate, not a measurement.
- No battery-backed RTC, so the clock needs NTP after a power loss.
- BLE support is limited to scanning and a basic GATT connection.

## License

Released under the MIT License. See [LICENSE](LICENSE).
