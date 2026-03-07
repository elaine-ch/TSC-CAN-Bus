void setup() {
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}

void loop() {

  if (digitalRead(2) == HIGH) {
    
    precharge();

    while(digitalRead(2) == HIGH){
      digitalWrite(4, HIGH);
    }

  } else {
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
}

void precharge() {
  // Turn pin 3 on and wait 2.5 sec
    digitalWrite(3, HIGH);
    delay(2500); 

    // Turn pin 3 off, wait 0.5 sec
    digitalWrite(3, LOW);
    delay(500);
}