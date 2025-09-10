SETU: Safety & Emergency Tracking Unit 
An Offline Communication & Tracking Gadget for the Maha Kumbh Mela  Reconnecting Faith, Ensuring Safety.

Team ID : TH12744

1. Overview: SETU (Safety & Emergency Tracking Unit) is a portable offline communication and tracking device designed for the Maha Kumbh Mela. It enables location sharing and emergency alerts without internet or mobile networks. The unit ensures quick coordination during crowd surges, lost-person incidents, or medical emergencies. By bridging technology and tradition, SETU reconnects faith while ensuring safety for millions of pilgrims.

2. Problem & Solution: In remote areas or large gatherings like the Kumbh Mela, network connectivity is unreliable or unavailable, making real-time communication and safety coordination difficult.
Our device provides long-range (2–5 km) point-to-point communication using LoRa technology.  It supports push-to-talk voice transfer, short text messages, GPS-based distance indication (LEDs),  and SOS alerts, ensuring reliable communication in zero-network conditions.

3. Logic & Workflow 
• Data Collection: - Voice captured via microphone, converted to digital via ESP32-S3 + I2S codec.
 - GPS module provides real-time coordinates.
- Text input through push button/serial trigger.
• Processing: - ESP32-S3 encodes audio/text and wraps with destination address + channel.
  - LoRa module transmits point-to-point in fixed mode.
• Output: - Peer device receives data and outputs voice via MAX98357A speaker driver.
  - Text and connection status displayed on OLED.
  - Distance indicated with Green/Orange/Red LEDs.
• User Side:  - Push-to-talk button for voice.
  - OLED for messages/status.
  - SOS button for emergency siren + location send.
• Admin Side: - Can track deployed devices via GPS logs.
  - Can manage device IDs and pairing.

4. Tech Stack
• ESP32-S3 (Microcontroller + Wi-Fi + BLE)
• LoRa AS32-TTL-100 (Long range RF module)
• OLED Display (SSD1306, I²C)
• INMP441(I2S Digital Mic)
• MAX98357A (I2S Audio DAC & Amplifier)
• GPS Module (NEO-6M / compatible)
• Arduino IDE (Firmware Development)
• C/C++ (Embedded Code)

5. Future Scope
The prototype can be extended with:
• Mobile app integration for admin monitoring.
• Mesh-network support for multi-device communication.
• Encrypted communication for secure transmission.
• Integration with cloud dashboards for large-scale event management.