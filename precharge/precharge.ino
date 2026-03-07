void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}

void loop() {

  if (digitalRead(2) == HIGH) {
    // Serial.println("Beginning");
    precharge();

    while(digitalRead(2) == HIGH){
      Serial.println(digitalRead(2));
      digitalWrite(4, HIGH);
    }

  } else {
    Serial.println("Off");
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
  // Serial.println(digitalRead(2));
}

void precharge() {
  // Turn pin 3 on and wait 2.5 sec
    Serial.println("Starting precharge");
    digitalWrite(3, HIGH);
    delay(2500); 

    // Turn pin 3 off, wait 0.5 sec
    digitalWrite(3, LOW);
    delay(500);
}