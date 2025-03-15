#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30100_PulseOximeter.h"

// Constants
#define REPORTING_PERIOD_MS 1000
#define BUZZER_PIN PB12
#define X_AXIS_PIN PB0
#define Y_AXIS_PIN PB1
#define HEARTBEAT_THRESHOLD 120
#define FALL_THRESHOLD_LOW 1800
#define FALL_THRESHOLD_HIGH 2400

// Objects and variables
LiquidCrystal_I2C lcd(0x27, 16, 2);
PulseOximeter pox;
uint32_t tsLastReport = 0;
float BPM, SpO2;
unsigned long lastTime = 0;

// Function prototypes
void Init_spo2();
void Heartbeat_Check();
void Fall_Check();
void onBeatDetected();

void setup() {
    // Initialize peripherals
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.begin(9600);
    Serial2.begin(9600);

    lcd.init();
    lcd.backlight();

    // Display initialization message
    lcd.clear();
    lcd.print("Health Monitoring");
    lcd.setCursor(0, 1);
    lcd.print("    System..");
    delay(2000);

    // Initialize pulse oximeter
    if (!pox.begin()) {
        Serial.println("Pulse Oximeter Initialization FAILED");
        while (true);  // Stay in loop if initialization fails
    }
    Serial.println("Pulse Oximeter Initialized");

    pox.setOnBeatDetectedCallback(onBeatDetected);
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
}

void loop() {
    Heartbeat_Check();
    Fall_Check();
}

void Init_spo2() {
    Serial.print("Reinitializing Pulse Oximeter...");
    if (!pox.begin()) {
        Serial.println("FAILED");
        while (true);
    }
    Serial.println("SUCCESS");
    pox.setOnBeatDetectedCallback(onBeatDetected);
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
}

void Heartbeat_Check() {
    Init_spo2();
    for (int i = 0; i < 10000; i++) {
        pox.update();
        BPM = pox.getHeartRate();
        SpO2 = pox.getSpO2();

        if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
            // Log and display heart rate and SpO2
            Serial.print("HB: ");
            Serial.println(BPM);
            Serial.print("SpO2: ");
            Serial.print(SpO2);
            Serial.println("%");

            Serial2.print("HB: ");
            Serial2.println(BPM);
            Serial2.print("SpO2: ");
            Serial2.print(SpO2);
            Serial2.println("%");

            lcd.clear();
            lcd.print("HB: ");
            lcd.print(BPM);
            lcd.setCursor(0, 1);
            lcd.print("SpO2: ");
            lcd.print(SpO2);
            lcd.print("%");

            tsLastReport = millis();
        }

        if (BPM > HEARTBEAT_THRESHOLD) {
            // Trigger high heart rate alert
            lcd.clear();
            lcd.print("High Heartbeat!");
            Serial.println("High Heartbeat Detected");
            Serial2.println("High Heartbeat Detected");
            digitalWrite(BUZZER_PIN, HIGH);
            delay(2000);
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}

void Fall_Check() {
    int X_val = analogRead(X_AXIS_PIN);
    int Y_val = analogRead(Y_AXIS_PIN);

    // Display accelerometer readings
    lcd.clear();
    lcd.print("X: ");
    lcd.print(X_val);
    lcd.setCursor(0, 1);
    lcd.print("Y: ");
    lcd.print(Y_val);
    Serial.print("X: ");
    Serial.println(X_val);
    Serial.print("Y: ");
    Serial.println(Y_val);

    delay(1000);

    if (X_val < FALL_THRESHOLD_LOW || X_val > FALL_THRESHOLD_HIGH ||
        Y_val < FALL_THRESHOLD_LOW || Y_val > FALL_THRESHOLD_HIGH) {
        // Trigger fall alert
        lcd.clear();
        lcd.print("Fall Detected");
        Serial.println("Fall Detected");
        Serial2.println("Fall Detected");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(2000);
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void onBeatDetected() {
    Serial.println("Beat Detected!");
}
