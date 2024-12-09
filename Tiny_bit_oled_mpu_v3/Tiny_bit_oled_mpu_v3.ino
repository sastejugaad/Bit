#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "DataParser.h"
#include "Cdrv8833.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Compass.h"
#include "HeadingController.h"
#include "MPU9250.h"

MPU9250 mpu;

HeadingController headingController;

// OLED display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin not used, can be set to -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Initialize Compass
Compass compass(&display);

#define WIFI_SSID "ground"
#define WIFI_PASSWORD "12345678"
#define UDP_PORT 12345

// Motor Pins
#define IN1_PIN 4
#define IN2_PIN 3
#define RCHANNEL 0
#define IN3_PIN 0
#define IN4_PIN 2
#define LCHANNEL 1

DataParser dataParser;  // Create an instance of DataParser
int Speed = 20;
int Right_speed = 0;
int Left_speed = 0;
String movement;

WiFiUDP udp;
Cdrv8833 LMotor;
Cdrv8833 RMotor;

float initialHeading = 0;  // Initial heading when heading keeping is enabled
bool headingKeepingEnabled = false;

float globalYaw = 1.0;
float globalPitch = 1.0;
float globalRoll = 1.0;

int display_data = 0;

void setup() {
  Serial.begin(115200);

  // Initialize OLED display
  Wire.begin(6, 7); // SDA = 6, SCL = 7
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    //while (true);  // Stop execution if OLED init fails
  }
      if (!mpu.setup(0x68)) {  // change to your own address
       // while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
      //  }
    }
   delay(100);
  // Initialize Compass
  mpu.update();
  //delay(100);
  compass.init((mpu.getYaw()* (PI / 180.0))); // Set the initial yaw as "front"
  
  // Display startup message
  display.clearDisplay();
  display.setTextSize(1); // Normal 1x scale
  display.setTextColor(SSD1306_WHITE);
  display.println("Starting...");
  display.println("WiFi Connecting...");
  display.display();
  
  // Initialize Motors
  RMotor.init(IN1_PIN, IN2_PIN, LCHANNEL, false);
  LMotor.init(IN3_PIN, IN4_PIN, RCHANNEL, false);
  
  // Initialize HeadingController with default PID values
  headingController.setPID(1, 0, 0.1);
 // Default PID values, can be overridden by user

  // Connect to WiFi
  connectToWiFi();

  // Start UDP
  udp.begin(UDP_PORT);
  Serial.println("UDP Listening on port ");
   delay(5000);
  calibrate_imu();
  
}

void loop() {
    if (mpu.update()) {
        mpu.update();

        globalYaw = mpu.getYaw();   // Current yaw angle
        globalRoll = mpu.getPitch();
        globalPitch = mpu.getRoll();

        // Display based on user input
        if (display_data == 1) {
            compass.drawCompass(-mpu.getYaw() * (PI / 180.0));
        } else if (display_data == 2) {
            compass.drawCube(mpu.getPitch() * (PI / 180.0), 
                             mpu.getYaw() * (PI / 180.0), 
                             mpu.getRoll() * (PI / 180.0));
        } else {
            displayIMUValues();
        }
    }

    // Maintain heading while moving or stationary
    if (movement == "f" || movement == "b") {
        // If moving, maintain heading within ±2 degrees
        keepHeading(globalYaw, true);  // `true` indicates moving
    } else if (movement == "s") {
        // If stopped, maintain heading
        keepHeading(globalYaw, false); // `false` indicates stationary
    } else {
        headingController.reset(); // Reset PID when turning or idle
        headingController.setTargetHeading(globalYaw); // Update heading
    }

    udpReceiveTask();
}


void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // Needed for ESP32-C3
  Serial.print("Connecting to WiFi");
  
  // Update OLED during WiFi connection attempts
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connecting...");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.print(".");
    display.display();
  }

  // Update OLED upon successful connection
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected!");
  display.println("SSID: " + String(WIFI_SSID));
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.print("Port: ");
  display.println(UDP_PORT);
  display.display();
}

void udpReceiveTask() {
  char packetBuffer[255];
//Serial.println("udp");
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    
    if (len > 0) {
      packetBuffer[len] = 0;  // Null-terminate the string
      Serial.println("Received UDP packet:");
      Serial.println(packetBuffer);

      String IncomingData = packetBuffer;

      // Parse data using DataParser
      dataParser.parseData(IncomingData, ','); // Pass data and delimiter

      Speed = (dataParser.getField(1)).toInt();
                  // Update PID gains from user input
      float kp = (dataParser.getField(2)).toFloat();
      float ki = (dataParser.getField(3)).toFloat();
      float kd = (dataParser.getField(4)).toFloat();
      display_data = (dataParser.getField(5)).toInt();
      
      headingController.setPID(kp, ki, kd);
      
      Left_speed = Speed;
      Right_speed = Speed;
      movement = (dataParser.getField(0));


      // Execute movement based on the parsed data
      if (movement == "f") {
  
        forward();
      } else if (movement == "b") {
 
        backward();
      } else if (movement == "l") {
        left();
      } else if (movement == "r") {
        right();
      } else if (movement == "s") {
       Stop();
      }
    }
  }
  delay(10);  // Small delay to avoid CPU overload
}

void motor_speed(int Right_Speed, int Left_Speed) {
  Left_speed = Left_Speed;
  Right_speed = Right_Speed;
}

void forward() {
  //Serial.print("fwd");
  LMotor.move(Left_speed);
  RMotor.move(Right_speed);
}

void backward() {
  LMotor.move(-Left_speed);
  RMotor.move(-Right_speed);
}

void left() {
  LMotor.move(Left_speed);
  RMotor.move(-Right_speed);
}

void right() {
  LMotor.move(-Left_speed);
  RMotor.move(Right_speed);
}

void Stop() {
  LMotor.move(0);
  RMotor.move(0);
}


void keepHeading(float currentHeading, bool isMoving) {
    // Compute PID correction
    float pidOutput = headingController.computePID(currentHeading);

    if (isMoving) {
        // Add PID correction to motor speeds to maintain straight path
        int leftSpeed = constrain(Speed - pidOutput, -100, 100);
        int rightSpeed = constrain(Speed + pidOutput, -100, 100);

        LMotor.move(leftSpeed);
        RMotor.move(rightSpeed);
    } else {
        // Adjust motors to maintain heading while stationary
        int leftSpeed = constrain(-pidOutput, -100, 100);
        int rightSpeed = constrain(pidOutput, -100, 100);

        LMotor.move(leftSpeed);
        RMotor.move(rightSpeed);
    }
}


void displayIMUValues() {
  // Get IMU values
  mpu.update();
  float pitch =mpu.getPitch(); 
  float yaw = mpu.getYaw();
  float roll = mpu.getRoll();

  // Update OLED with IMU values
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);  // Use 1x scale
  display.println("IMU Values:");
  display.print("Pitch: ");
  display.println(pitch, 1); // Display with 1 decimal place
  display.print("Yaw: ");
  display.println(yaw, 1);
  display.print("Roll: ");
  display.println(roll, 1);
  display.display();
}

void calibrate_imu()
{   
      // Update OLED with IMU values
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);  // Use 1x scale
    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.Please leave the device still on the flat plane.");
    display.println("Accel Gyro calibration will start in 5sec.Place on the flat plane.");
    mpu.verbose(true);
     display.display();
    delay(5000);
    mpu.calibrateAccelGyro();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);  // Use 1x scale
    
    Serial.println("Mag calibration will start in 5sec.Please Wave device in a figure eight until done.");
    display.println("Mag calibration will start in 5sec.Please Wave device in a figure eight until done.");
    display.display();
    delay(5000);
    mpu.calibrateMag();
    print_calibration();
    mpu.verbose(false);  
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
