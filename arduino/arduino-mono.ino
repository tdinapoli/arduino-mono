
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);  // Match this baud rate in Python
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // Read command line
    int pin, n, onTime, offTime, state;
    if (cmd.startsWith("READ")){
      pin = cmd.substring(5).toInt();
      int state = digitalRead(pin);
      Serial.println(state == HIGH ? "HIGH" : "LOW");
    }
    else if (sscanf(cmd.c_str(), "%d %d %d %d", &pin, &n, &onTime, &offTime) == 4) {  
      togglePinNTimes(pin, n, onTime, offTime);
      Serial.println("OK");
    } 
    else if (sscanf(cmd.c_str(), "%d %d", &pin, &state) == 2){
      digitalWrite(pin, state);
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
  }
}

void togglePinNTimes(int pin, int N, unsigned long onTime, unsigned long offTime) {
  for (int i = 0; i < N; i++) {
    digitalWrite(pin, HIGH);
    delay(onTime);
    digitalWrite(pin, LOW);
    delay(offTime);
  }
}
