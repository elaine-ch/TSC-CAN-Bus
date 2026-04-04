void setup() {
  Serial.begin(9600);

  //control
  pinMode(2, INPUT);
  //precharge enable
  pinMode(3, OUTPUT);
  //charge enable
  pinMode(4, OUTPUT);

  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}

void loop() {

  //read discharge enable from bms, start precharging when BMS output goes from 12V -> 0V
  if (digitalRead(2) == LOW) {
    // Serial.println("Beginning");
    precharge();

    while(digitalRead(2) == LOW){
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