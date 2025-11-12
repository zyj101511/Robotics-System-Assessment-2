#define EMIT_PIN 11
#define NUM_SENSORS 1

float reading = 0;
float result;

int received_count = 0;
byte received_byte = 0;
unsigned long last_change_ts = 0;

bool last_state = true; //前车亮

void setup() {
  pinMode(EMIT_PIN, INPUT);
  pinMode(A6, INPUT_PULLUP);
  // digitalWrite(EMIT_PIN, HIGH);  // 打开红外发射灯
  Serial.begin(9600);
  delay(1500);
}

void loop() {
  reading = analogRead(A6);
  bool current_state = (reading < 600);

  unsigned long now = micros();
  
  int bit_value = -1; // -1 表示本轮没检测到新bit

  if(current_state != last_state && now - last_change_ts > 1300){
    last_change_ts = now;
    bit_value = current_state ? 0 : 1;
  }

  if(bit_value != -1) {
    received_byte = (received_byte << 1) | bit_value;
    received_count++;

    if(received_count >= 8) {
      Serial.print("Received byte: ");
      Serial.println(received_byte, BIN);
      received_count = 0;
      received_byte = 0;
    }
  }
  last_state = current_state;

}
