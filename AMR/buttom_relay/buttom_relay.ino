int start = 19;
int power = 17;
int emergency = 15;
int relay_led_R = 16;
int relay_led_G = 22;
int relay_led_B = 21;
void setup()
{
    pinMode(start , INPUT_PULLUP);
    pinMode(power , INPUT_PULLUP);
    pinMode(emergency , INPUT_PULLUP);
    Serial.begin(115200);
    pinMode(relay_led_R, OUTPUT);
    pinMode(relay_led_G, OUTPUT);
    pinMode(relay_led_B, OUTPUT);
}
void loop()
{
    delay(250);

    // Read the states of the pins
    int startState = digitalRead(start);
    int powerState = digitalRead(power);
    int emergencyState = digitalRead(emergency);

    // Print the states to Serial Monitor
    Serial.print("Start: ");
    Serial.println(startState == LOW ? "ON" : "OFF");

    Serial.print("Power: ");
    Serial.println(powerState == LOW ? "ON" : "OFF");

    Serial.print("Emergency: ");
    Serial.println(emergencyState == LOW ? "ON" : "OFF");

    // Add a separator for readability
    Serial.println("--------------------");

    // Check signals and control LEDs
    if (digitalRead(power) == LOW && digitalRead(start) == LOW && digitalRead(emergency) == LOW) {
        digitalWrite(relay_led_G, HIGH);
        digitalWrite(relay_led_R, LOW);        
    } else if (digitalRead(power) == LOW && digitalRead(start) == HIGH && digitalRead(emergency) == LOW) {
        digitalWrite(relay_led_G, HIGH);
        digitalWrite(relay_led_R, HIGH);
    } else {
        digitalWrite(relay_led_R, HIGH);
        digitalWrite(relay_led_G, LOW); 

    }

    // Delay for readability
    delay(1000);
}
