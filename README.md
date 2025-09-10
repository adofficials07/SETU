# SETU
This project is a portable two-way communication device built on ESP32-S3 with LoRa modules.  It enables long-range voice and text communication without internet or cellular networks,  supports GPS-based location tracking, OLED-based status display, and SOS alert. It is designed for disaster management, large events, and remote area communication.
## Features

*   **Two-Way Voice Communication:** Half-duplex voice transmission between two paired devices using a push-to-talk button.
*   **Location Tracking:** On-demand tracking of a paired device's GPS location. Displays distance and bearing to the target.
*   **Proximity Indication (RGB LED):**
    *   **Red:** Target > 2 km away
    *   **Blue:** Target < 1 km away
    *   **Green:** Target < 300 meters away (very near)
*   **SOS Alert System:** Dedicated button to send an emergency alert with current location to the paired device, activating a local siren and red LED indicator on both devices.
*   **OLED Display:** Provides real-time status updates (Talking, Listening, SOS, Location Tracking, Distance, Bearing).
*   **Device Pairing:** Communication is restricted to two specific, paired devices using unique IDs.
*   **Modular Design:** Utilizes external LoRa, GPS, and Magnetometer modules, I2S audio, and an OLED display.

## Hardware Components

*   **2x ESP32-S3 DevKitC-1** (or similar ESP32-S3 development board)
*   **2x AS32-TTL-100 LoRa Module**
*   **2x I2S Digital Microphone** (e.g., INMP441, SPH0645)
*   **2x MAX98357A I2S Amplifier**
*   **2x NEO-7M GPS Module**
*   **2x GY-271 Magnetometer Module** (HMC5883L/QMC5883L)
*   **2x 0.96" I2C OLED Display** (SSD1306)
*   **2x Push Buttons** (for Talk, Location, SOS)
*   **2x Piezo Buzzer** (for SOS siren)
*   **2x Red LED** (for SOS indicator, in addition to onboard RGB LED)
*   **2x Speaker** (4Ω - 8Ω, compatible with MAX98357A)
*   **Connecting Wires, Breadboards/Protoboards**

## Software / Technology Stack

*   **Microcontroller:** ESP32-S3
*   **IDE:** Arduino IDE
*   **Programming Language:** C++
*   **Libraries:**
    *   `HardwareSerial.h` (Built-in for UART communication)
    *   `driver/i2s.h` (ESP-IDF for I2S Audio)
    *   `Wire.h` (Built-in for I2C communication)
    *   `U8g2lib.h` (for OLED Display)
    *   `TinyGPSPlus.h` (for GPS data parsing)
    *   `Adafruit_Sensor.h` (Base for sensor libraries)
    *   `Adafruit_HMC5883_U.h` (for HMC5883L magnetometers like GY-271) - *Note: If your GY-271 uses QMC5883L, you'll need a different library like `QMC5883L.h`.*
    *   `FastLED.h` (for onboard WS2812B RGB LED)

## Wiring Instructions (ESP32-S3 DevKitC-1 Example)

**Critical Note:** Verify pin assignments against your *specific* ESP32-S3 board's pinout and module datasheets.

### Shared Connections (All Modules)
*   **3.3V:** Connect all VCC pins of modules to ESP32-S3 3.3V.
*   **GND:** Connect all GND pins of modules to ESP32-S3 GND.

### I2C Bus (OLED, Magnetometer)
*   **ESP32-S3 GPIO8 (SDA)** <-> OLED SDA, GY-271 SDA
*   **ESP32-S3 GPIO9 (SCL)** <-> OLED SCL, GY-271 SCL

### UART2 (LoRa Module)
*   **ESP32-S3 GPIO17 (RX2)** <-> AS32-TTL-100 TX
*   **ESP32-S3 GPIO18 (TX2)** <-> AS32-TTL-100 RX

### UART1 (GPS Module)
*   **ESP32-S3 GPIO20 (RX1)** <-> NEO-7M TX
*   **ESP32-S3 GPIO21 (TX1)** <-> NEO-7M RX
*   *Note: If your GPS module requires a 3.3V logic level, ensure it's compatible or use a logic level converter if it's 5V.*

### I2S Audio (Mic & Speaker)
*   **ESP32-S3 GPIO13 (BCLK)** <-> I2S Mic SCK, MAX98357A BCLK
*   **ESP32-S3 GPIO14 (LRCLK)** <-> I2S Mic WS, MAX98357A LRCLK
*   **ESP32-S3 GPIO15 (DIN)** <-> I2S Mic SD
*   **ESP32-S3 GPIO16 (DOUT)** <-> MAX98357A DIN
*   **ESP32-S3 GPIO4** <-> MAX98357A SD (Shutdown, tie to 3.3V if always on)

### Digital Inputs/Outputs
*   **ESP32-S3 GPIO5** <-> Push Button 1 (Talk) - other side to GND
*   **ESP32-S3 GPIO6** <-> Push Button 2 (Location) - other side to GND
*   **ESP32-S3 GPIO7** <-> Push Button 3 (SOS) - other side to GND
*   **ESP32-S3 GPIO48** (or specific pin for your onboard RGB LED) <-> RGB LED (WS2812B data line)
*   **ESP32-S3 GPIO1** <-> Piezo Buzzer (other side to GND)
*   **ESP32-S3 GPIO2** <-> Red LED (SOS) - with current limiting resistor (e.g., 220 Ohm) - other side to GND
