/*
 * APEX 0 - DIY ExpressLRS Radio Transmitter Handset
 * Developed for Hack Club Macondo Project
 * * Target MCU: ESP32 30-Pin DevKit
 * Mappings perfectly matched to Final Transmitter Schematic
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Hardware Pin Definitions ---
#define PIN_JOY_LEFT_X  34  // LRx (ADC1)
#define PIN_JOY_LEFT_Y  35  // LRy (ADC1)
#define PIN_SWITCH_LEFT 25  // LSW (Digital Input)

#define PIN_JOY_RIGHT_X 32  // RRx (ADC1)
#define PIN_JOY_RIGHT_Y 33  // RRy (ADC1)
#define PIN_SWITCH_RIGHT 26 // RSW (Digital Input)

#define PIN_POT_AUX     36  // VP (ADC1 Tuning Potentiometer)

#define PIN_UART_TX2    17  // Connected to ELRS RX Pad
#define PIN_UART_RX2    16  // Connected to ELRS TX Pad

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- CRSF Frame Structures ---
#define CRSF_START_BYTE     0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16

struct CrsfChannels {
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11;
    unsigned int ch6 : 11;
    unsigned int ch7 : 11;
    unsigned int ch8 : 11;
    unsigned int ch9 : 11;
    unsigned int ch10 : 11;
    unsigned int ch11 : 11;
    unsigned int ch12 : 11;
    unsigned int ch13 : 11;
    unsigned int ch14 : 11;
    unsigned int ch15 : 11;
} __attribute__((packed));

uint8_t crsfPacket[26];

// --- CRC8 Calculation Function for ExpressLRS ---
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void setup() {
    // Initialize Debug Console
    Serial.begin(115200);
    
    // Initialize Hardware UART2 for ExpressLRS Module at native CRSF Speed
    Serial2.begin(416666, SERIAL_8N1, PIN_UART_RX2, PIN_UART_TX2);
    
    // Configure Switches with internal pullups
    pinMode(PIN_SWITCH_LEFT, INPUT_PULLUP);
    pinMode(PIN_SWITCH_RIGHT, INPUT_PULLUP);
    
    // Initialize I2C OLED Screen
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED allocation failed"));
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.println("  APEX 0 TX ONLINE  ");
        display.println("   HACKCLUB MACONDO ");
        display.display();
    }
}

void loop() {
    // 1. Read Analog Joysticks and Pot (ESP32 12-bit ADC: 0 to 4095)
    int raw_roll     = analogRead(PIN_JOY_RIGHT_X);
    int raw_pitch    = analogRead(PIN_JOY_RIGHT_Y);
    int raw_throttle = analogRead(PIN_JOY_LEFT_Y);
    int raw_yaw      = analogRead(PIN_JOY_LEFT_X);
    int raw_aux      = analogRead(PIN_POT_AUX);
    
    // 2. Map values to standard ExpressLRS channel limits (CRSF ranges from 172 to 1811)
    // Center point is 992. Equivalent to standard 1000us - 2000us servo signals.
    uint16_t crsf_roll     = map(raw_roll,     0, 4095, 172, 1811);
    uint16_t crsf_pitch    = map(raw_pitch,    0, 4095, 172, 1811);
    uint16_t crsf_throttle = map(raw_throttle, 0, 4095, 172, 1811);
    uint16_t crsf_yaw      = map(raw_yaw,      0, 4095, 172, 1811);
    uint16_t crsf_aux      = map(raw_aux,      0, 4095, 172, 1811);
    
    uint16_t sw_l = (digitalRead(PIN_SWITCH_LEFT) == LOW) ? 1811 : 172;
    uint16_t sw_r = (digitalRead(PIN_SWITCH_RIGHT) == LOW) ? 1811 : 172;

    // 3. Assemble the CRSF Serialization Packet Array
    crsfPacket[0] = CRSF_START_BYTE;
    crsfPacket[1] = 24; // Packet Payload Length
    crsfPacket[2] = CRSF_FRAMETYPE_RC_CHANNELS;
    
    CrsfChannels *channels = (CrsfChannels *)&crsfPacket[3];
    channels->ch0 = crsf_roll;
    channels->ch1 = crsf_pitch;
    channels->ch2 = crsf_throttle;
    channels->ch3 = crsf_yaw;
    channels->ch4 = sw_l;
    channels->ch5 = sw_r;
    channels->ch6 = crsf_aux;
    
    // Calculate Checksum over payload and payload header type
    crsfPacket[25] = crsf_crc8(&crsfPacket[2], 23);
    
    // 4. Stream data frame down the serial pipeline directly to the ELRS module
    Serial2.write(crsfPacket, 26);
    
    // 5. Refresh OLED Telemetry Screen Layout at a clean 10Hz pace
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 100) {
        lastUpdate = millis();
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("--- APEX 0 LINK ---");
        display.printf("THR: %d\n", map(crsf_throttle, 172, 1811, 0, 100));
        display.printf("RLL: %d | PTH: %d\n", map(crsf_roll, 172, 1811, -50, 50), map(crsf_pitch, 172, 1811, -50, 50));
        display.printf("AUX POT: %d\n", map(crsf_aux, 172, 1811, 0, 100));
        display.println("STATUS: WAITING RX...");
        display.display();
    }
    
    delay(20); // Balanced 50Hz control loop interval
}