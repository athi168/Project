#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <HCSR04.h> 

// Servo X, Y (joystick)
Servo x_servo;
Servo y_servo;

int x_servoPin = 32;
int y_servoPin = 33;

// Cảm biến siêu âm
#define TRIG_FRONT 25
#define ECHO_FRONT 26
#define TRIG_LEFT  12
#define ECHO_LEFT  14
#define TRIG_RIGHT 27
#define ECHO_RIGHT 34

UltraSonicDistanceSensor ultra_front(TRIG_FRONT, ECHO_FRONT);
UltraSonicDistanceSensor ultra_left(TRIG_LEFT, ECHO_LEFT);
UltraSonicDistanceSensor ultra_right(TRIG_RIGHT, ECHO_RIGHT);

// Cấu trúc dữ liệu nhận từ tay cầm
typedef struct data_structure {
  int x;
  int y;
  bool isButtonPressed;
} data_structure;

data_structure receivedData;

// Đọc và xử lý cảm biến trong chế độ tự hành
void handleAutonomousMode() {
  float d_front = ultra_front.measureDistanceCm();
  float d_left = ultra_left.measureDistanceCm();
  float d_right = ultra_right.measureDistanceCm();

  Serial.print("Front (25,26): ");
  if (d_front > 0) Serial.println(String(d_front) + " cm"); else Serial.println("No echo");

  Serial.print("Left (12,14): ");
  if (d_left > 0) Serial.println(String(d_left) + " cm"); else Serial.println("No echo");

  Serial.print("Right (27,34): ");
  if (d_right > 0) Serial.println(String(d_right) + " cm"); else Serial.println("No echo");

  // Logic điều khiển servo Y trong chế độ tự hành
  if(d_front > 0.0 && d_front < 25.0){
    if (d_left > 0 && d_left < 15.0){
      Serial.println("Vật cản bên trái");
      y_servo.write(30);  
    }
    else if (d_right > 0 && d_right < 15.0) {
      Serial.println("Vật cản bên phải");
      y_servo.write(150);
    }
  }
    else if(d_front > 0.0 && d_front < 25.0){
      x_servo.writeMicroseconds(1000);   
      y_servo.write(90); 
    }
  else {
    y_servo.write(90);  // không có vật cản → giữ trung tâm
  }
}

// Callback khi nhận dữ liệu
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  Serial.print("x: "); Serial.println(receivedData.x);
  Serial.print("y: "); Serial.println(receivedData.y);
  Serial.print("isButtonPressed: "); Serial.println(receivedData.isButtonPressed);
  Serial.println();

  if (receivedData.isButtonPressed == LOW) {
    Serial.println("AUTO MODE: motor X quay 1570");
    x_servo.writeMicroseconds(1600);
  } else if (receivedData.isButtonPressed == HIGH){
    Serial.println("MANUAL MODE: motor X theo joystick");
    x_servo.writeMicroseconds(receivedData.x);

    // Điều khiển servo Y theo joystick
    y_servo.write(90 + float(receivedData.y));
  }
}

void setup() {
  Serial.begin(115200);

  // Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  x_servo.setPeriodHertz(100);
  x_servo.attach(x_servoPin, 1000, 2000);

  y_servo.setPeriodHertz(100);
  y_servo.attach(y_servoPin, 1000, 2400);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (receivedData.isButtonPressed == LOW) {
    handleAutonomousMode();  // xử lý cảm biến & điều khiển servo Y
  }

  delay(500);  // đọc mỗi 0.5 giây
}
