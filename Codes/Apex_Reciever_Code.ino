/*
 * APEX 0 - DIY Flight Control Autopilot Software
 * Developed for Hack Club Macondo Project
 * * Target MCU: ESP32 30-Pin DevKit
 * Mappings perfectly matched to Final Receiver Perfboard Layout
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Hardware Pin Definitions ---
#define PIN_UART_RX2     16 // Connected to ELRS TX Pad
#define PIN_UART_TX2     17 // Connected to ELRS RX Pad

#define PIN_PWM_THROTTLE 25 // J1: ESC Command
#define PIN_PWM_AIL_L    26 // J2: Left Aileron Servo
#define PIN_PWM_AIL_R    27 // J3: Right Aileron Servo
#define PIN_PWM_ELEVATOR 14 // J4: Elevator Servo
#define PIN_PWM_RUDDER   13 // J5: Rudder Servo

// --- LEDC Hardware PWM Channel Configuration ---
#define PWM_FREQ         50    // Standard analog servo frequency (50Hz)
#define PWM_RESOLUTION   16    // 16-bit resolution tracking (0 to 65535)

// 50Hz period = 20ms. 
// 1ms pulse width (Min RC range)  = (1ms / 20ms) * 65535 = 3276
// 1.5ms pulse width (Center)      = (1.5ms / 20ms) * 65535 = 4915
// 2ms pulse width (Max RC range)  = (2ms / 20ms) * 65535 = 6553

#define RC_MIN_DUTY      3276
#define RC_CEN_DUTY      4915
#define RC_MAX_DUTY      6553

// Define independent internal hardware generation timers
#define CH_THROTTLE      0
#define CH_AIL_L         1
#define CH_AIL_R         2
#define CH_ELEVATOR      3
#define CH_RUDDER        4

// --- Core Autopilot System Variables ---
Adafruit_MPU6050 mpu;
int16_t channels[4] = {1500, 1500, 1000, 1500}; // Roll, Pitch, Throttle, Yaw Defaults

// --- PID Control Variable Structures ---
float roll_error = 0, pitch_error = 0;
float roll_pwm_output = 0, pitch_pwm_output = 0;

// PID Tunings (Configured for standard foam park-fliers; adjustable post-flight trials)
float Kp = 1.2;
float Ki = 0.05;
float Kd = 0.4;

void setup() {
    Serial.begin(115200);
    
    // Initialize ExpressLRS UART2 Stream Data Port
    Serial2.begin(416666, SERIAL_8N1, PIN_UART_RX2, PIN_UART_TX2);
    
    // Initialize MPU6050 / 10DOF Frame Interface 
    if (!mpu.begin()) {
        Serial.println("MPU6050 baseline connection offline. Proceeding in testing bypass mode.");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    
    // Configure ESP32 LEDC Timers & Pins for Servos & Motor Outputs
    ledcSetup(CH_THROTTLE, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_THROTTLE, CH_THROTTLE);
    
    ledcSetup(CH_AIL_L, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_AIL_L, CH_AIL_L);
    
    ledcSetup(CH_AIL_R, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_AIL_R, CH_AIL_R);
    
    ledcSetup(CH_ELEVATOR, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_ELEVATOR, CH_ELEVATOR);
    
    ledcSetup(CH_RUDDER, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_RUDDER, CH_RUDDER);
    
    // Safe-Start Initialization (Force Throttle Low to avoid motor accidental spooling)
    ledcWrite(CH_THROTTLE, RC_MIN_DUTY);
}

void loop() {
    // 1. Check for Incoming CRSF Packets from ELRS Module
    if (Serial2.available() >= 26) {
        if (Serial2.read() == 0xC8) { // Locate CRSF Sync Start Byte
            uint8_t buffer[25];
            Serial2.readBytes(buffer, 25);
            
            if (buffer[1] == 0x16) { // Verify Channels Frame Type
                // Decode packed 11-bit chunks back into regular raw channel integer metrics
                channels[0] = ((buffer[2] | buffer[3] << 8) & 0x07FF);        // Roll Channel
                channels[1] = ((buffer[3] >> 3 | buffer[4] << 5) & 0x07FF);   // Pitch Channel
                channels[2] = ((buffer[4] >> 6 | buffer[5] << 2 | buffer[6] << 10) & 0x07FF); // Throttle
                channels[3] = ((buffer[6] >> 1 | buffer[7] << 7) & 0x07FF);   // Yaw Channel
            }
        }
    }
    
    // 2. Fetch Inertial Readouts from the IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate simple Euler angles from raw IMU metrics (Complementary Filter Skeleton)
    float pitch_actual = (atan2(a.acceleration.y, a.acceleration.z) * 180.0) / M_PI;
    float roll_actual  = (atan2(a.acceleration.x, a.acceleration.z) * 180.0) / M_PI;
    
    // Convert stick data to target angles (-30 to +30 degrees bank limits)
    float roll_target  = map(channels[0], 172, 1811, -30, 30);
    float pitch_target = map(channels[1], 172, 1811, -30, 30);
    
    // 3. Execution of Autonomous PID Stability Loop calculations
    roll_error = roll_target - roll_actual;
    pitch_error = pitch_target - pitch_actual;
    
    roll_pwm_output  = roll_error * Kp;   // Basic Proportional stabilization response
    pitch_pwm_output = pitch_error * Kp;
    
    // 4. Actuator Mixer Engine Map Matrix
    // Map intermediate calculation metrics straight back down to standard LEDC Hardware Outputs
    long throttle_duty = map(channels[2], 172, 1811, RC_MIN_DUTY, RC_MAX_DUTY);
    long rudder_duty   = map(channels[3], 172, 1811, RC_MIN_DUTY, RC_MAX_DUTY);
    
    long ail_l_duty    = RC_CEN_DUTY + (roll_pwm_output * 50); // Direct Differential Mix
    long ail_r_duty    = RC_CEN_DUTY - (roll_pwm_output * 50); // Ailerons act in opposite orientations
    long elevator_duty = RC_CEN_DUTY + (pitch_pwm_output * 50);
    
    // Enforce bounds protection clips to avoid mechanically stripping micro servo gears
    ail_l_duty    = constrain(ail_l_duty, RC_MIN_DUTY, RC_MAX_DUTY);
    ail_r_duty    = constrain(ail_r_duty, RC_MIN_DUTY, RC_MAX_DUTY);
    elevator_duty = constrain(elevator_duty, RC_MIN_DUTY, RC_MAX_DUTY);
    
    // Write out the mixed calculations straight to the physical hardware pins
    ledcWrite(CH_THROTTLE, throttle_duty);
    ledcWrite(CH_AIL_L, ail_l_duty);
    ledcWrite(CH_AIL_R, ail_r_duty);
    ledcWrite(CH_ELEVATOR, elevator_duty);
    ledcWrite(CH_RUDDER, rudder_duty);
    
    delay(10); // High performance, low latency fixed 100Hz autopilot update execution loop
}