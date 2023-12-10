#include <TinyGPSPlus.h>

#include <SoftwareSerial.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define GPS_RX 10
#define GPS_TX 11
#define SPEAKER_PIN 9
#define MB1240_ANALOG_PIN A0

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

#define I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define THRESHOLD_DISTANCE 200
#define SPEED_THRESHOLD 5.0

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  lcd.begin(LCD_COLS, LCD_ROWS);
  pinMode(SPEAKER_PIN, OUTPUT);
  lcd.print("Vehicle Detection");
}

void loop() {
  // MB1240 sensor logic (Condition Two)
  int sensorValue = analogRead(MB1240_ANALOG_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = voltage * 100.0; // Example calibration for a specific sensor

  if (distance < THRESHOLD_DISTANCE) {  // Adjust the threshold for 2 meters
    // Display the information on the LCD for Condition Two
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Mode Circulation ");
    lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    lcd.print(distance);
    lcd.print(" cm");

    playSound(" mode Circulation ");
    delay(5000);  // Display the message and sound for 5 seconds
    lcd.clear();
    lcd.print("Vehicle Detection");
  } else {
    // GPS logic
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.speed.kmph() > SPEED_THRESHOLD) {  // Adjust the GPS threshold based on your needs
          // Display the information on the LCD for GPS speed check
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Vehicle Detected");
          lcd.setCursor(0, 1);
          lcd.print("Speed: ");
          lcd.print(gps.speed.kmph());
          lcd.print(" km/h");

          playSound("Vehicle Detected");
          delay(5000);  // Display the message and sound for 5 seconds
          lcd.clear();
          lcd.print("Vehicle Detection");
        }
      }
    }
  }
}

void playSound(String message) {
  tone(SPEAKER_PIN, 1000, 1000);  // Assuming a 1kHz tone for 1 second
  delay(1000);  // Wait for 1 second
  noTone(SPEAKER_PIN);
}
