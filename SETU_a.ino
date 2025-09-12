/*  
 * ----------------------------------------------------------------------------
 *  Project:  SETU - ESP32-S3 LoRa PTT Voice + SMS + GPS + SOS Communication
 *  Hardware: ESP32-S3, SX1278 LoRa, INMP441 Mic, MAX98357A DAC, Neo-7M GPS,
 *            HMC5883/QMC5883 Magnetometer, SSD1306 OLED, WS2812/NeoPixel LED
 * 
 *  Description:
 *    This code implements Push-to-Talk (PTT) voice with ADPCM compression,
 *    SMS-style text fallback, GPS location sharing, and SOS siren + alerts
 *    between two ESP32-S3 devices over LoRa.
 * 
 *  Author:   Shyam Agarwal (c) 2025  
 *  Company:  AGARWAL EXPLORATION & INNOVATION PVT LTD
 * 
 *  License:  MIT License
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  1. The above copyright notice, company name, and this permission notice shall
 *     be included in all copies or substantial portions of the Software.
 *  2. This copyright header must NOT be removed from this file in any copy or
 *     derivative work.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * ----------------------------------------------------------------------------
 */


/* ---------- LIBRARIES ---------- */
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <Adafruit_NeoPixel.h>
#include "driver/i2s.h"

/* ---------- USER CONFIG (change if needed) ---------- */
// Addresses (set per device)
const uint16_t MY_ADDR   = 0x0001;   // device A = 0x0001, device B = 0x0002
const uint16_t PEER_ADDR = 0x0002;

// LoRa SPI pins (VSPI3 mapping you gave)
#define LORA_SCK   39
#define LORA_MISO  36
#define LORA_MOSI  37
#define LORA_CS    35
#define LORA_RST   38
#define LORA_DIO0  7

const long LORA_FREQ = 433E6;  // change to 865E6 for India legal band if module supports
const int  LORA_SF   = 9;
const long LORA_BW   = 125E3;
const int  LORA_CR   = 5;

// I2S mic (INMP441) pins
#define I2S_MIC_BCLK 4
#define I2S_MIC_WS   5
#define I2S_MIC_SD   6

// I2S DAC (MAX98357A)
#define I2S_DAC_BCLK 21
#define I2S_DAC_WS   19
#define I2S_DAC_DIN  20

// GPS (Serial1) pins - adjust if needed
#define GPS_RX_PIN 18  // GPS TX -> ESP RX
#define GPS_TX_PIN 17  // GPS RX <- ESP TX

// Magnetometer (HMC5883-like) I2C address
#define MAG_ADDR 0x1E

// NeoPixel (single RGB)
#define NEO_PIN 48
Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Buttons
#define PTT_BTN 0   // talk (hold)
#define LOC_BTN 12  // send location / request
#define SOS_BTN 13  // send SOS

// Other params
#define SAMPLE_RATE 8000
#define I2S_BUF_SAMPLES 256

/* ---------- Globals ---------- */
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

unsigned long lastRx = 0;
bool paired = false;
String last_msg = "";
int last_rssi = 0, last_snr = 0;
// I2C (OLED, magnetometer)
#define I2C_SDA 8
#define I2C_SCL 9

/* ---------- ADPCM (IMA) Implementation ---------- */
static const int indexTable[16] = {
 -1, -1, -1, -1, 2, 4, 6, 8,
 -1, -1, -1, -1, 2, 4, 6, 8
};
static const int stepTable[89] = {
 7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,
 34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,
 157,173,190,209,230,253,279,307,337,371,408,449,494,544,
 598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,
 2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,
 6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,
 18500,20350,22385,24623,27086,29794,32767
};

struct ADPCMState { int valprev; int index; };
ADPCMState txState, rxState;
void ADPCM_init(ADPCMState &s){ s.valprev=0; s.index=0; }

/* encode PCM (int16) pairs to IMA ADPCM 4-bit (packed into bytes)
   pcm_len = number of samples (should be even); returns bytes produced */
int ADPCM_encode_block(ADPCMState &state, const int16_t *pcm_in, int pcm_len, uint8_t *out4bit){
  int out_i=0; int valprev=state.valprev; int index=state.index;
  for(int i=0;i<pcm_len;i+=2){
    int codes[2];
    for(int j=0;j<2;j++){
      int sample = pcm_in[i+j];
      int step = stepTable[index];
      int diff = sample - valprev;
      int sign = 0; if (diff < 0) { sign = 8; diff = -diff; }
      int delta = 0;
      int temp = step;
      if (diff >= temp) { delta |= 4; diff -= temp; }
      temp >>= 1; if (diff >= temp) { delta |= 2; diff -= temp; }
      temp >>= 1; if (diff >= temp) { delta |= 1; }
      int diffq = step >> 3;
      if (delta & 4) diffq += step;
      if (delta & 2) diffq += step >> 1;
      if (delta & 1) diffq += step >> 2;
      if (sign) valprev -= diffq; else valprev += diffq;
      if (valprev > 32767) valprev = 32767; if (valprev < -32768) valprev = -32768;
      index += indexTable[delta | sign];
      if (index < 0) index = 0; if (index > 88) index = 88;
      codes[j] = (delta | sign) & 0xF;
    }
    out4bit[out_i++] = (uint8_t)((codes[0] << 4) | (codes[1] & 0xF));
  }
  state.valprev = valprev; state.index = index;
  return out_i;
}

/* decode ADPCM bytes -> PCM int16 samples; returns number of samples generated */
int ADPCM_decode_block(ADPCMState &state, const uint8_t *in4bit, int in_len, int16_t *pcm_out){
  int out_s = 0; int valprev = state.valprev; int index = state.index;
  for (int i=0;i<in_len;i++){
    uint8_t b = in4bit[i];
    for (int s=0;s<2;s++){
      int code = (s==0) ? ((b>>4)&0xF) : (b & 0xF);
      int step = stepTable[index];
      int diffq = step >> 3;
      if (code & 4) diffq += step;
      if (code & 2) diffq += step >> 1;
      if (code & 1) diffq += step >> 2;
      if (code & 8) valprev -= diffq; else valprev += diffq;
      if (valprev > 32767) valprev = 32767; if (valprev < -32768) valprev = -32768;
      index += indexTable[code];
      if (index < 0) index = 0; if (index > 88) index = 88;
      pcm_out[out_s++] = (int16_t)valprev;
    }
  }
  state.valprev = valprev; state.index = index;
  return out_s;
}

/* ---------- I2S MIC / DAC init ---------- */
#define I2S_NUM_MIC I2S_NUM_0
#define I2S_NUM_DAC I2S_NUM_1

void i2s_mic_init() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_MIC_BCLK,
    .ws_io_num  = I2S_MIC_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_MIC_SD
  };
  i2s_driver_install(I2S_NUM_MIC, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_MIC, &pins);
}

void i2s_dac_init() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_DAC_BCLK,
    .ws_io_num  = I2S_DAC_WS,
    .data_out_num = I2S_DAC_DIN,
    .data_in_num = -1
  };
  i2s_driver_install(I2S_NUM_DAC, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_DAC, &pins);
}

/* ---------- Magnetometer basic init (HMC5883L-like) ---------- */
#define MAG_ADDR 0x1E
void mag_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  // Try HMC5883L init; if your module is QMC5883 or different you will need to adapt
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

float mag_read_heading() {
  uint8_t buf[6] = {0};
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, (uint8_t)6);
  for (int i=0;i<6 && Wire.available(); i++) buf[i] = Wire.read();
  int16_t x = (buf[0] << 8) | buf[1];
  int16_t z = (buf[2] << 8) | buf[3];
  int16_t y = (buf[4] << 8) | buf[5];
  float heading = atan2((float)y, (float)x) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

/* ---------- GPS ---------- */
void gps_init() {
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

/* ---------- Display, NeoPixel utility ---------- */
void showDistanceLED(double meters) {
  if (meters < 300) { pixel.setPixelColor(0, pixel.Color(0,150,0)); } // green
  else if (meters < 2000) { pixel.setPixelColor(0, pixel.Color(0,0,200)); } // blue
  else { pixel.setPixelColor(0, pixel.Color(200,0,0)); } // red
  pixel.show();
}

/* ---------- LoRa packet types ---------- */
#define TYPE_TEXT  0x01
#define TYPE_VOICE 0x02
#define TYPE_LOC   0x03
#define TYPE_SOS   0x04

/* ---------- Globals ---------- */


/* ---------- LoRa send helpers ---------- */
void sendTextPacket(uint16_t dst, const String &msg) {
  uint8_t hdr[6];
  hdr[0] = (dst >> 8) & 0xFF; hdr[1] = dst & 0xFF;
  hdr[2] = (MY_ADDR >> 8) & 0xFF; hdr[3] = MY_ADDR & 0xFF;
  hdr[4] = TYPE_TEXT; hdr[5] = (uint8_t)msg.length();
  LoRa.beginPacket();
  LoRa.write(hdr, 6);
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("Sent TXT: " + msg);delay(1000);
}

/* Send a compressed ADPCM voice chunk (pcm_samples length samples, even) */
void sendVoiceChunk(uint16_t dst, const int16_t *pcm_samples, int pcm_samples_len) {
  // pcm_samples_len must be even
  if (pcm_samples_len <= 0) return;
  uint8_t comp[220]; // safe for pcm_len up to ~440 samples (compLen = pcm_len/2)
  int compLen = ADPCM_encode_block(txState, pcm_samples, pcm_samples_len, comp);
  if (compLen <= 0) return;
  if (compLen > 200) compLen = 200; // safety clamp
  uint8_t hdr[6];
  hdr[0] = (dst >> 8) & 0xFF; hdr[1] = dst & 0xFF;
  hdr[2] = (MY_ADDR >> 8) & 0xFF; hdr[3] = MY_ADDR & 0xFF;
  hdr[4] = TYPE_VOICE; hdr[5] = (uint8_t)compLen;
  LoRa.beginPacket();
  LoRa.write(hdr, 6);
  LoRa.write(comp, compLen);
  LoRa.endPacket();
}

/* Send location (lat,lng string) */
void sendLocationPacket(uint16_t dst) {
  if (!gps.location.isValid() || gps.location.age() > 2000) {
    display.clearDisplay(); display.setCursor(0,0);
    display.println("GPS: NO FIX");
    display.display();
    return;
  }
  String payload = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  uint8_t hdr[6];
  hdr[0] = (dst >> 8) & 0xFF; hdr[1] = dst & 0xFF;
  hdr[2] = (MY_ADDR >> 8) & 0xFF; hdr[3] = MY_ADDR & 0xFF;
  hdr[4] = TYPE_LOC; hdr[5] = (uint8_t)payload.length();
  LoRa.beginPacket(); LoRa.write(hdr,6); LoRa.print(payload); LoRa.endPacket();
  Serial.println("Sent LOC: " + payload);
}

/* Play decoded PCM (samples) through I2S DAC */
void playPCMAndFlush(int16_t *pcm, int samples) {
  if (samples <= 0) return;
  // write in blocks if needed
  size_t written;
  i2s_write(I2S_NUM_DAC, pcm, samples * sizeof(int16_t), &written, portMAX_DELAY);
}

/* Play silence (multiple blocks) to ensure DAC stops */
void flushSilence() {
  int16_t silence[256] = {0};
  size_t written;
  for (int i=0;i<6;i++) {
    i2s_write(I2S_NUM_DAC, silence, sizeof(silence), &written, portMAX_DELAY);
    delay(5);
  }
}

/* Simple distance bearing helpers */
double haversine_distance_m(double lat1,double lon1,double lat2,double lon2){
  double R = 6371000.0;
  double dLat = (lat2-lat1) * DEG_TO_RAD;
  double dLon = (lon2-lon1) * DEG_TO_RAD;
  double a = sin(dLat/2)*sin(dLat/2) + cos(lat1*DEG_TO_RAD)*cos(lat2*DEG_TO_RAD)*sin(dLon/2)*sin(dLon/2);
  double c = 2*atan2(sqrt(a), sqrt(1-a));
  return R * c;
}
double bearing_between(double lat1,double lon1,double lat2,double lon2){
  double phi1 = lat1 * DEG_TO_RAD; double phi2 = lat2 * DEG_TO_RAD;
  double dl = (lon2 - lon1) * DEG_TO_RAD;
  double y = sin(dl) * cos(phi2);
  double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dl);
  double theta = atan2(y,x) * 180.0/PI;
  if (theta < 0) theta += 360.0;
  return theta;
}

/* OLED status update */
void showStatus(const String &line1="", const String &line2="") {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("Me:0x"); display.println(MY_ADDR, HEX);
  display.print("Peer:0x"); display.println(PEER_ADDR, HEX);
  display.print("State:"); display.println(paired ? "Paired":"Waiting");
  if (line1.length()) {
    display.println();
    display.println(line1);
  }
  if (line2.length()) display.println(line2);
  display.display();
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);

  // I2C for OLED & magnetometer
  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // NeoPixel
  pixel.begin(); pixel.show();

  // ADPCM init
  ADPCM_init(txState); ADPCM_init(rxState);

  // I2S init
  i2s_mic_init(); i2s_dac_init();

  // GPS + magnetometer
  gps_init(); mag_init();

  // LoRa SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    showStatus("LoRa init failed","");
    while (1);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(20);

  pinMode(PTT_BTN, INPUT_PULLUP);
  pinMode(LOC_BTN, INPUT_PULLUP);
  pinMode(SOS_BTN, INPUT_PULLUP);

  showStatus("Ready","");
}

/* ---------- MAIN LOOP ---------- */
void loop() {
  // Read GPS stream
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  // Check incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize >= 6) {
    uint8_t hdr[6];
    LoRa.readBytes(hdr, 6);
    uint16_t dst = (hdr[0] << 8) | hdr[1];
    uint16_t src = (hdr[2] << 8) | hdr[3];
    uint8_t type = hdr[4];
    uint8_t len  = hdr[5];
    int n = min((int)len, packetSize - 6);
    uint8_t payload[256];
    if (n > 0) LoRa.readBytes(payload, n);
    payload[n] = 0;
    last_rssi = LoRa.packetRssi();
    last_snr  = (int)LoRa.packetSnr();

    if (dst == MY_ADDR || dst == 0xFFFF) {
      lastRx = millis();
      if (src == PEER_ADDR) paired = true;

      if (type == TYPE_TEXT) {
        String s = String((char*)payload);
        last_msg = s;
        showStatus("TXT from 0x" + String(src, HEX), s);
        Serial.println("TXT RX: " + s);delay(1000);
      }
      else if (type == TYPE_LOC) {
        String s = String((char*)payload);
        if (!gps.location.isValid()) {
          showStatus("LOC rx", "GPS: NO FIX");
        } else {
          int comma = s.indexOf(',');
          if (comma > 0) {
            double plat = s.substring(0,comma).toDouble();
            double plon = s.substring(comma+1).toDouble();
            double meters = haversine_distance_m(gps.location.lat(), gps.location.lng(), plat, plon);
            double bear = bearing_between(gps.location.lat(), gps.location.lng(), plat, plon);
            float heading = mag_read_heading();
            double delta = bear - heading;
            if (delta < -180) delta += 360; if (delta > 180) delta -= 360;
            showStatus("LOC from 0x" + String(src, HEX), "dist(m):" + String((int)meters));
            display.println("bear:" + String((int)bear) + " head:" + String((int)heading));
            display.display();
            showDistanceLED(meters);
          } else {
            showStatus("LOC rx", "parse fail");
          }
        }
      }
      else if (type == TYPE_VOICE) {
        // decode & play
        uint8_t *comp = payload;
        int compLen = n;
        int16_t pcmOut[512];
        int outSamples = ADPCM_decode_block(rxState, comp, compLen, pcmOut);

        // apply simple gain
        for (int i=0;i<outSamples;i++) {
          int val = pcmOut[i] * 2; // gain x2
          if (val > 32767) val = 32767; if (val < -32768) val = -32768;
          pcmOut[i] = (int16_t)val;
        }
        playPCMAndFlush(pcmOut, outSamples);
        // do not flash siren for incoming voice
        showStatus("VOICE RX from 0x" + String(src, HEX), "RSSI:" + String(last_rssi));
      }
      else if (type == TYPE_SOS) {
        // show only, do NOT play siren on receiver
        showStatus("SOS from 0x" + String(src, HEX), "");
        Serial.println("SOS RX from 0x" + String(src, HEX));
      }
    }
  }

  // clear paired flag if timeout
  if (millis() - lastRx > 15000) paired = false;

  // BUTTONS: PTT (voice)
  if (digitalRead(PTT_BTN) == LOW) {
    // announce
    sendTextPacket(PEER_ADDR, "VOICE start 📢 I am here, can you hear me?Need help,");
    showStatus("PTT sending...", "");

    // record & stream in blocks
    int16_t pcm[I2S_BUF_SAMPLES];
    size_t bytesRead = 0;

    while (digitalRead(PTT_BTN) == LOW) {
      // read PCM from mic
      i2s_read(I2S_NUM_MIC, (void*)pcm, sizeof(pcm), &bytesRead, portMAX_DELAY);
      int samples = bytesRead / sizeof(int16_t);
      if (samples % 2) samples--;
      if (samples > 0) {
        sendVoiceChunk(PEER_ADDR, pcm, samples);
      }
    }

    // flush local audio
    flushSilence();
    // notify end
    sendTextPacket(PEER_ADDR, "VOICE end🆗 Over");
    showStatus("PTT ended","");
  }

  // LOC button: send location
  if (digitalRead(LOC_BTN) == LOW) {
    sendLocationPacket(PEER_ADDR);
    delay(500); // debounce
  }

  // SOS button: local siren + send text only (peer shows text)
  if (digitalRead(SOS_BTN) == LOW) {
    // local siren (non-blocking-ish, small bursts)
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
      int16_t buf[256];
      for (int i=0;i<256;i++) {
        float t = (float)i / (float)SAMPLE_RATE;
        float freq = 700 + 400.0 * ((millis()-t0)/(float)4000.0); // sweep 700->1100Hz
        buf[i] = (int16_t)(sin(2.0 * PI * freq * t) * 12000);
      }
      size_t written;
      i2s_write(I2S_NUM_DAC, buf, sizeof(buf), &written, portMAX_DELAY);
      // blink red
      pixel.setPixelColor(0, pixel.Color(200,0,0)); pixel.show();
      delay(100);
      pixel.clear(); pixel.show();
      delay(50);
    }
    flushSilence();
    // send SOS message to peer (text)
    sendTextPacket(PEER_ADDR, "SOS need emergency from 0x" + String(MY_ADDR, HEX));
    delay(500);
  }

  // Serial input to send custom text
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) sendTextPacket(PEER_ADDR, s);
  }

  // small UI refresh
  static unsigned long lastUi = 0;
  if (millis() - lastUi > 800) {
    showStatus();
    lastUi = millis();
  }
}
