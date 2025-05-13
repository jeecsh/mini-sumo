#include <driver/ledc.h>
#include <PS4Controller.h>

// Pin Definitions
#define IN1 32
#define IN2 33
#define ENA 27
#define IN3 25
#define IN4 26
#define ENB 14
#define TRIG_PIN 23
#define ECHO_PIN 22
#define R_SENSOR 35
#define L_SENSOR 34

// Constants
#define LEDC_BASE_FREQ 5000
#define MAX_DISTANCE 400
#define MIN_DISTANCE 2
#define FULL_SPEED 255
#define SEARCH_SPEED 180
#define ATTACK_SPEED 255
#define TURN_SPEED 200
#define OPPONENT_DETECTION_DISTANCE 40
#define LINE_DETECTED LOW

// Global Variables
float distance = 0;
float duration = 0;
bool isRunning = false;
bool isConnected = false;
unsigned long lastSearchDirectionChange = 0;
bool searchingClockwise = true;
unsigned long searchStartTime = 0;
unsigned long lastConnectionCheck = 0;

// Function declarations (prototypes)
void configureLEDC();
void stopMotors();
void setMotorSpeed(uint8_t channel, uint8_t speed);
float getDistance();
bool isLineDetected();
void search();
void attack();
void retreat();
void rotate(bool clockwise);
void moveForward(uint8_t speed);
void moveBackward(uint8_t speed);
void onConnect();
void onDisconnect();

// Function implementations
void stopMotors() {
  setMotorSpeed(LEDC_CHANNEL_0, 0);
  setMotorSpeed(LEDC_CHANNEL_1, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotorSpeed(uint8_t channel, uint8_t speed) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  return (duration / 2) * 0.0343;
}

bool isLineDetected() {
  return (digitalRead(R_SENSOR) == LINE_DETECTED || 
          digitalRead(L_SENSOR) == LINE_DETECTED);
}

void search() {
  if (millis() - lastSearchDirectionChange > 2000) {
    searchingClockwise = !searchingClockwise;
    lastSearchDirectionChange = millis();
  }
  rotate(searchingClockwise);
}

void attack() {
  Serial.println("Attacking!");
  moveForward(ATTACK_SPEED);
}

void retreat() {
  Serial.println("Retreating!");
  moveBackward(FULL_SPEED);
  delay(200);
  rotate(searchingClockwise);
  delay(300);
}

void rotate(bool clockwise) {
  if (clockwise) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  setMotorSpeed(LEDC_CHANNEL_0, TURN_SPEED);
  setMotorSpeed(LEDC_CHANNEL_1, TURN_SPEED);
}

void moveForward(uint8_t speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(LEDC_CHANNEL_0, speed);
  setMotorSpeed(LEDC_CHANNEL_1, speed);
}

void moveBackward(uint8_t speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorSpeed(LEDC_CHANNEL_0, speed);
  setMotorSpeed(LEDC_CHANNEL_1, speed);
}

void configureLEDC() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = LEDC_BASE_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel_A = {
    .gpio_num = ENA,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_A);

  ledc_channel_config_t ledc_channel_B = {
    .gpio_num = ENB,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_B);
}

void onConnect() {
  Serial.println("PS4 Controller Connected!");
  isConnected = true;
}

void onDisconnect() {
  Serial.println("PS4 Controller Disconnected!");
  isConnected = false;
  isRunning = false;
  stopMotors();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Mini Sumo Robot Starting...");
  
  // Configure pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(R_SENSOR, INPUT);
  pinMode(L_SENSOR, INPUT);
  
  // Configure LEDC for PWM
  configureLEDC();
  
  // Initialize PS4 controller
  Serial.println("Starting PS4 controller initialization...");
  if(!PS4.begin("84:30:95:78:C7:DB")) {
    Serial.println("Failed to initialize PS4 controller!");
    return;
  }
  Serial.println("PS4 controller initialized. Waiting for connection...");
  
  // Event handlers for PS4 controller
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  
  Serial.println("Setup complete. Ready for operation.");
}

void loop() {
  // Connection status check every 2 seconds
  if (millis() - lastConnectionCheck > 2000) {
    if (PS4.isConnected()) {
      Serial.println("Controller Status: Connected");
      Serial.printf("Battery Level: %d%%\n", PS4.Battery());
    } else {
      Serial.println("Controller Status: Not Connected");
    }
    lastConnectionCheck = millis();
  }
  
  if (PS4.isConnected()) {
    if (PS4.Cross()) {
      Serial.println("Cross Button Pressed - Starting");
      isRunning = true;
      searchStartTime = millis();
    }
    if (PS4.Square()) {
      Serial.println("Square Button Pressed - Stopping");
      isRunning = false;
      stopMotors();
    }
    
    if (isRunning) {
      distance = getDistance();
      Serial.printf("Distance: %.2f cm\n", distance);
      
      if (isLineDetected()) {
        Serial.println("Line detected!");
        retreat();
      }
      else if (distance >= MIN_DISTANCE && distance <= OPPONENT_DETECTION_DISTANCE) {
        Serial.println("Opponent detected!");
        attack();
      }
      else {
        search();
      }
    }
  } else {
    if (isRunning) {
      Serial.println("Controller disconnected - stopping motors");
      isRunning = false;
      stopMotors();
    }
  }
  
  delay(50);
}
