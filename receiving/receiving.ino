#define THRESHOLD 530
#define BIT_INTERVAL 1300
#define FRAME_TIMEOUT 10000  
#define EMIT_PIN 11

float reading;
bool current_state = false;
bool last_state = false;

byte received_byte = 0;
int received_count = 0;
bool receiving = false;
unsigned long last_change_ts = 0;

void setup() {
  
  pinMode(EMIT_PIN, INPUT);
  pinMode(A6, INPUT_PULLUP);
  
  Serial.begin(9600);
  delay(1500);
}

void loop() {
  reading = analogRead(A6);
  current_state = (reading < THRESHOLD);
  unsigned long now = micros();

  if (now - last_change_ts > FRAME_TIMEOUT) {
    receiving = false;
    received_count = 0;
    received_byte = 0;
    last_change_ts = now;
  }

  if (current_state != last_state && now - last_change_ts > BIT_INTERVAL) {
    last_change_ts = now;
    int bit_value = current_state ? 0 : 1;  

    if (!receiving) {
 
      if (bit_value == 1) {
        receiving = true;
        received_byte = (received_byte << 1) | bit_value;
        received_count++;
      }
    } 
    
    else {
      received_byte = (received_byte << 1) | bit_value;
      received_count++;

      if (bit_value == 0 && received_count >= 8) {
        Serial.print("Byte received: ");
        Serial.println(received_byte, BIN);
        receiving = false;
        received_byte = 0;
        received_count = 0;
      }

    }
  }

  last_state = current_state;
}
