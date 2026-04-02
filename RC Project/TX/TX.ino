#include <WiFi.h>
#include <esp_now.h>


// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x08, 0xA6, 0xF7, 0x48, 0x43, 0x3C}; 

// Khai báo peerInfo (gửi dữ liệu qua ESP-NOW)
esp_now_peer_info_t peerInfo;

// ==========================================================================
typedef struct data_structure {
    long x;
    long y;
    char signal;
    bool isButtonPressed;
} data_structure;

data_structure sending_data;
const int JOY_STICK_VRX = 16;
const int JOY_STICK_VRY = 17;
const int PUSH_BUTTON = 18;

// Callback khi dữ liệu được gửi
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    char macStr[18];
    Serial.print("Packet to: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    pinMode(JOY_STICK_VRX, INPUT);
    pinMode(JOY_STICK_VRY, INPUT);
    pinMode(PUSH_BUTTON, INPUT_PULLUP);
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        while (100); // Dừng nếu lỗi
    }

    esp_now_register_send_cb(OnDataSent);

    // Khởi tạo và thêm peer
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        while (100);
    }
}

void loop() {
    float x_axis = analogRead(JOY_STICK_VRX);
    float y_axis = analogRead(JOY_STICK_VRY);
    int sw = digitalRead(PUSH_BUTTON);
    int axis_max = 4095;

    // Ánh xạ giá trị joystick
    int x = map(x_axis, 0, 4095, 750, 2000); // 1600 Phù hợp với writeMicroseconds
    float y = map(y_axis, 0, 4095, -65, 50);  // Phù hợp với servo.write(90 + y)
    Serial.print("X_axis: ");
    Serial.println(x);
    Serial.print("Y-axis: ");
    Serial.println(y);
    Serial.print("SW: ");
    Serial.println(sw);

    sending_data.isButtonPressed = sw;
    sending_data.signal = 0xff;
    sending_data.x = x;
    sending_data.y = y;

    // Gửi dữ liệu tới RX
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&sending_data, sizeof(data_structure));
    if (result == ESP_OK) {
        Serial.println("Sent with success");

    } else {
        Serial.println("Error sending the data");
    }
        delay(100);
}

