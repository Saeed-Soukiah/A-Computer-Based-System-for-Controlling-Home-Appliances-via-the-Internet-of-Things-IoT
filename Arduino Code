/*
  Computer System For Controlling House Devices
  Created In: 28/11/2022 1:13:47 AM
  Modified: 18 June 2025 (Optimized using timers instead of delay())
  Author: Saeed Soukiah
  Microcontroller Board: Arduino Mega 2560
*/

// -------------------------
// Library Inclusions & Serial Definitions
// -------------------------
#include <Keypad.h>
#include <Stepper.h>
#include <Wire.h>
#include "RTClib.h"
#include <Nextion.h>
#include "HX711.h"

// Use hardware serial ports on Arduino Mega:
#define GSMSerial        Serial1    // GSM Module: RX1 (pin 19), TX1 (pin 18)
#define WIFI_SERIAL      Serial2    // WiFi Module: RX2 (pin 16), TX2 (pin 17)
#define NEXTION_SERIAL   Serial3    // Nextion Display: RX3 (pin 15), TX3 (pin 14)

// -------------------------
// Global Variables and Constants
// -------------------------

// Keypad Setup
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {22, 23, 24, 25};
byte colPins[COLS] = {26, 27, 28};
Keypad mykeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Stepper Motor (Feeder)
const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, 8, 7, 6, 5);

// Fish Feeder Timing (8 hours interval)
unsigned long previousFeederMillis = 0;
const unsigned long feederInterval = 28800000UL; // 8 hours

// WiFi & Security
byte WIFI_Lock = 0;
char WIFICommand = 0;

// Device Pin Definitions
#define LED_PIN           53
#define LED1_PIN          52
#define BREAK_ALARM_PIN   13
#define FIRE_SENSOR_PIN   14
#define FIRE_ALARM_PIN    48    // Fire alarm pin remains at 48
#define CARBON_SENSOR_PIN 47
#define CARBON_ALARM_PIN  49    // Carbon alarm reassigned to pin 49
#define WINDOW_PIN        51
#define DOOR_PIN          36
#define OUTDOOR_PIN       35    // IR-triggered output (e.g., outdoor light/lock)
#define ROOF_PIN          37    // Roof control reassigned to pin 37

// IR Sensors for In/Out Counter
#define IR1_PIN           7
#define IR2_PIN           8
int peopleCount = 0;
bool ir1State = true;
bool ir2State = true;
int irSequence = 1; // Determines the sensor sequence for entry/exit

// Debounce Settings for IR Sensors
unsigned long lastIR1Time = 0;
unsigned long lastIR2Time = 0;
const unsigned long debounceDelay = 50;

// RTC
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Nextion Display and Buttons
int CurrentPage = 0;
NexButton b0 = NexButton(0, 1, "b0");
NexButton b1 = NexButton(0, 2, "b1");
NexButton b2 = NexButton(0, 3, "b2");
NexButton b3 = NexButton(0, 4, "b3");
NexButton b4 = NexButton(0, 5, "b4");
NexButton b5 = NexButton(0, 6, "b5");
NexButton b6 = NexButton(0, 7, "b6");
NexButton b7 = NexButton(0, 8, "b7");
NexButton b8 = NexButton(0, 9, "b8");
NexTouch *nex_listen_list[] = { &b0, &b1, &b2, &b3, &b4, &b5, &b6, &b7, &b8, NULL };

// Load Cell (HX711)
HX711 Scale(A2, A3);
float loadValue = 0;

// Temperature Sensor (LM35) on Analog Pin A1

// SMS and Alarm Timing with State Machine
unsigned long previousSmsMillis = 0;
const unsigned long smsInterval = 5000UL; // Throttle SMS sending interval

enum SMSTransmitState { SMS_IDLE, SMS_SET_TEXT_MODE, SMS_SEND_NUMBER, SMS_SEND_MESSAGE, SMS_SEND_END };
SMSTransmitState smsState = SMS_IDLE;
unsigned long smsStateStartTime = 0;

// Non-Blocking Keypad Password Buffer
char passwordBuffer[5] = "";   // 4-digit password + null terminator
byte passwordIndex = 0;
byte failAttempts = 0;

// -------------------------
// Function Prototypes
// -------------------------
void processIRCounter();
void processFishFeeder(unsigned long now);
void processSwitches();
void processWiFiCommands();
void processFireAndCarbonSensors(unsigned long now);
void processSMS();
void processTempMotor();
void updateNextionDisplay();
void processKeypad();
void processLoadCell();
void move_motor(char Direction, byte Speed);

// Nextion Callback Prototypes
void page1PushCallback(void *ptr);
void b0PushCallback(void *ptr);
void b1PushCallback(void *ptr);
void b2PushCallback(void *ptr);
void b3PushCallback(void *ptr);
void b4PushCallback(void *ptr);
void b5PushCallback(void *ptr);
void b7PushCallback(void *ptr);
void b8PushCallback(void *ptr);

// -------------------------
// Setup Function
// -------------------------
void setup() {
  // Initialize Hardware Serial Ports
  GSMSerial.begin(9600);
  WIFI_SERIAL.begin(9600);
  NEXTION_SERIAL.begin(9600);
  
  // Set Pin Modes for Outputs and Inputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(BREAK_ALARM_PIN, OUTPUT);
  digitalWrite(BREAK_ALARM_PIN, LOW);
  
  pinMode(FIRE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(FIRE_ALARM_PIN, OUTPUT);
  
  pinMode(CARBON_SENSOR_PIN, INPUT_PULLUP);
  pinMode(CARBON_ALARM_PIN, OUTPUT);
  
  pinMode(WINDOW_PIN, OUTPUT);
  pinMode(DOOR_PIN, OUTPUT);
  digitalWrite(DOOR_PIN, HIGH); // Assume HIGH means door is unlocked
  
  pinMode(OUTDOOR_PIN, OUTPUT);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  
  pinMode(ROOF_PIN, OUTPUT);
  
  // Light switches (with internal pull-ups)
  pinMode(46, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);
  pinMode(43, INPUT_PULLUP);
  
  // RTC Setup
  rtc.begin();
  rtc.adjust(DateTime(2022, 11, 16, 4, 27, 0)); // Set initial date & time
  
  // Load Cell Calibration
  Scale.set_scale(415.f);
  
  // Attach Nextion Button Callbacks
  b0.attachPush(b0PushCallback);
  b1.attachPush(b1PushCallback);
  b2.attachPush(b2PushCallback);
  b3.attachPush(b3PushCallback);
  b4.attachPush(b4PushCallback);
  b5.attachPush(b5PushCallback);
  b6.attachPush(NULL); // Unassigned (or define if needed)
  b7.attachPush(b7PushCallback);
  b8.attachPush(b8PushCallback);
}

// -------------------------
// Main Loop Function
// -------------------------
void loop() {
  unsigned long now = millis();
  
  processIRCounter();
  processFishFeeder(now);
  processSwitches();
  processWiFiCommands();
  processFireAndCarbonSensors(now);
  processTempMotor();
  updateNextionDisplay();
  processKeypad();
  processLoadCell();
  
  nexLoop(nex_listen_list);  // Poll Nextion touch events
}

// -------------------------
// Function Definitions
// -------------------------

// 1. Process People Counter via IR Sensors (with non-blocking debounce)
void processIRCounter() {
  unsigned long now = millis();
  if (!digitalRead(IR1_PIN) && irSequence == 1 && ir1State) {
    if (now - lastIR1Time >= debounceDelay) {
      irSequence = 2;
      ir1State = false;
      lastIR1Time = now;
    }
  }
  else if (!digitalRead(IR2_PIN) && irSequence == 2 && ir2State) {
    if (now - lastIR2Time >= debounceDelay) {
      irSequence = 1;
      peopleCount++;
      ir2State = false;
      lastIR2Time = now;
    }
  }
  else if (!digitalRead(IR2_PIN) && irSequence == 1 && ir2State) {
    if (now - lastIR2Time >= debounceDelay) {
      irSequence = 2;
      ir2State = false;
      lastIR2Time = now;
    }
  }
  else if (!digitalRead(IR1_PIN) && irSequence == 2 && ir1State) {
    if (now - lastIR1Time >= debounceDelay) {
      peopleCount--;
      irSequence = 1;
      ir1State = false;
      lastIR1Time = now;
    }
  }
  if (digitalRead(IR1_PIN)) { ir1State = true; }
  if (digitalRead(IR2_PIN)) { ir2State = true; }
  
  if (peopleCount > 0 && WIFI_Lock == 1) {
    digitalWrite(OUTDOOR_PIN, HIGH);
  } else {
    digitalWrite(OUTDOOR_PIN, LOW);
  }
}

// 2. Process Fish Feeder (Stepper Motor) without blocking delays
void processFishFeeder(unsigned long now) {
  if (now - previousFeederMillis >= feederInterval) {
    previousFeederMillis = now;
    myStepper.step(146); // Advance one well (assumes 14 wells)
    // Disable stepper outputs to save power and minimize heating
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
}

// 3. Process Physical Switches (e.g., lights and window)
void processSwitches() {
  bool s1 = (digitalRead(46) == LOW);
  bool s2 = (digitalRead(45) == LOW);
  // Digital pin 44 is measured below for window control within updateNextionDisplay
  digitalWrite(LED_PIN, s1 ? HIGH : LOW);
  digitalWrite(LED1_PIN, s2 ? HIGH : LOW);
  digitalWrite(WINDOW_PIN, (digitalRead(44) == HIGH) ? HIGH : LOW);
}

// 4. Process WiFi Commands (non blocking)
void processWiFiCommands() {
  if (WIFI_SERIAL.available() > 0) {
    WIFICommand = WIFI_SERIAL.read();
  }
  switch (WIFICommand) {
    case 'A': WIFI_Lock = 1; break;
    case 'B': WIFI_Lock = 0; break;
    case 'E': if (WIFI_Lock == 1) digitalWrite(LED_PIN, HIGH); break;
    case 'F': if (WIFI_Lock == 1) digitalWrite(LED_PIN, LOW); break;
    case 'G': if (WIFI_Lock == 1) digitalWrite(LED1_PIN, HIGH); break;
    case 'H': if (WIFI_Lock == 1) digitalWrite(LED1_PIN, LOW); break;
    case 'I': if (WIFI_Lock == 1) move_motor('R', 64); break;
    case 'J': if (WIFI_Lock == 1) move_motor('R', 255); break;
    case 'K': if (WIFI_Lock == 1) move_motor('R', 0); break;
    case 'L': if (WIFI_Lock == 1) digitalWrite(WINDOW_PIN, HIGH); break;
    case 'M': if (WIFI_Lock == 1) digitalWrite(WINDOW_PIN, LOW); break;
    case 'O': if (WIFI_Lock == 1) digitalWrite(DOOR_PIN, HIGH); break;
    case 'P': if (WIFI_Lock == 1) digitalWrite(DOOR_PIN, LOW); break;
    default: break;
  }
  WIFICommand = 0; // Reset command after processing
}

// 5. Process Fire and Carbon Sensors & trigger SMS (uses non blocking SMS state machine)
void processFireAndCarbonSensors(unsigned long now) {
  bool fireTriggered = (digitalRead(FIRE_SENSOR_PIN) == LOW);
  bool carbonTriggered = (digitalRead(CARBON_SENSOR_PIN) == LOW);
  
  if (fireTriggered && carbonTriggered) {
    digitalWrite(FIRE_ALARM_PIN, HIGH);
    digitalWrite(CARBON_ALARM_PIN, HIGH);
    if (smsState == SMS_IDLE && now - previousSmsMillis >= smsInterval) {
      smsState = SMS_SET_TEXT_MODE;
      smsStateStartTime = now;
    }
  } else {
    digitalWrite(FIRE_ALARM_PIN, LOW);
    digitalWrite(CARBON_ALARM_PIN, LOW);
  }
  processSMS(); // Step through the SMS state machine if active
}

// Non blocking SMS state machine (each step uses a 50ms interval)
void processSMS() {
  unsigned long now = millis();
  switch(smsState) {
    case SMS_IDLE:
      break;
    case SMS_SET_TEXT_MODE:
      if (now - smsStateStartTime >= 50) {
        GSMSerial.println("AT+CMGF=1"); // Set SMS text mode
        smsState = SMS_SEND_NUMBER;
        smsStateStartTime = now;
      }
      break;
    case SMS_SEND_NUMBER:
      if (now - smsStateStartTime >= 50) {
        GSMSerial.println("AT+CMGS=\"+9631234567890\""); // Set recipient (update as needed)
        smsState = SMS_SEND_MESSAGE;
        smsStateStartTime = now;
      }
      break;
    case SMS_SEND_MESSAGE:
      if (now - smsStateStartTime >= 50) {
        GSMSerial.print("House On Fire !!!!!!");
        smsState = SMS_SEND_END;
        smsStateStartTime = now;
      }
      break;
    case SMS_SEND_END:
      if (now - smsStateStartTime >= 50) {
        GSMSerial.write(26);  // CTRL+Z to send SMS
        previousSmsMillis = now;
        smsState = SMS_IDLE;
      }
      break;
  }
}

// 6. Process Temperature Sensor (LM35) and Control Motor speed without blocking
void processTempMotor() {
  unsigned int sensorValue = analogRead(A1) / 2;  // Calibrated reading
  if (sensorValue >= 30) {
    move_motor('R', 255);
  } else if (sensorValue >= 25) {
    move_motor('R', 64);
  } else {
    move_motor('R', 0);
  }
}

// 7. Update Nextion Display (only on page 0, using Nextion terminator bytes)
void updateNextionDisplay() {
  if (CurrentPage == 0) {
    NEXTION_SERIAL.print("n0.val=");
    NEXTION_SERIAL.print(analogRead(A1) / 2);  // Sending the sensor value
    NEXTION_SERIAL.write(0xFF);
    NEXTION_SERIAL.write(0xFF);
    NEXTION_SERIAL.write(0xFF);
  }
}

// 8. Process Keypad Input for Security System (non blocking)
void processKeypad() {
  char key = mykeypad.getKey();
  if (key != NO_KEY) {
    passwordBuffer[passwordIndex++] = key;
    passwordBuffer[passwordIndex] = '\0'; // Null-terminate the string
    if (passwordIndex >= 4) {  // Check when 4 digits are entered
      if (strcmp(passwordBuffer, "1234") == 0) {
        digitalWrite(BREAK_ALARM_PIN, LOW);
        digitalWrite(DOOR_PIN, HIGH);
        WIFI_Lock = 1;
        failAttempts = 0;
      } else if (strcmp(passwordBuffer, "5678") == 0) {
        digitalWrite(DOOR_PIN, HIGH);
        WIFI_Lock = 0;
        failAttempts = 0;
      } else {
        failAttempts++;
      }
      // Reset password buffer:
      passwordIndex = 0;
      passwordBuffer[0] = '\0';
      // If too many failed attempts, trigger security:
      if (failAttempts >= 3) {
        digitalWrite(DOOR_PIN, LOW);
        digitalWrite(BREAK_ALARM_PIN, HIGH);
        WIFI_Lock = 0;
      }
    }
  }
}

// 9. Process Load Cell Reading and Control Roof Mechanism
void processLoadCell() {
  loadValue = Scale.read() / 100.0;
  if (loadValue == 0)
    digitalWrite(ROOF_PIN, HIGH);
  else
    digitalWrite(ROOF_PIN, LOW);
}

// Helper: Motor control function
void move_motor(char Direction, byte Speed) {
  analogWrite(11, Speed);  // PWM speed control on pin 11
  if (Direction == 'L') {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
  } else if (Direction == 'R') {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  } else if (Direction == 'S') {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
  }
}

// -------------------------
// Nextion Callback Functions
// -------------------------
void page1PushCallback(void *ptr) {
  CurrentPage = 1;
}
void b0PushCallback(void *ptr) {
  digitalWrite(LED_PIN, HIGH);
}
void b1PushCallback(void *ptr) {
  digitalWrite(LED_PIN, LOW);
}
void b2PushCallback(void *ptr) {
  digitalWrite(LED1_PIN, HIGH);
}
void b3PushCallback(void *ptr) {
  digitalWrite(LED1_PIN, LOW);
}
void b4PushCallback(void *ptr) {
  digitalWrite(WINDOW_PIN, HIGH);
}
void b5PushCallback(void *ptr) {
  digitalWrite(WINDOW_PIN, LOW);
}
void b7PushCallback(void *ptr) {
  digitalWrite(DOOR_PIN, HIGH);
}
void b8PushCallback(void *ptr) {
  digitalWrite(DOOR_PIN, HIGH);
}
