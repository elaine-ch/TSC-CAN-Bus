int BMS_IN = 2;
int PRE = 3;
int MAIN = 4;

void setup() {
  Serial.begin(9600);

  //control
  pinMode(BMS_IN, INPUT);
  //precharge enable
  pinMode(PRE, OUTPUT);
  //charge enable
  pinMode(MAIN, OUTPUT);

  digitalWrite(PRE, LOW);
  digitalWrite(MAIN, LOW);
}

void loop() {

  //read discharge enable from bms, start precharging when BMS output goes from 1BMS_INV -> 0V
  if (digitalRead(BMS_IN) == LOW) {
    // Serial.println("Beginning");
    precharge();

    while(digitalRead(BMS_IN) == LOW){
      Serial.println(digitalRead(BMS_IN));
      digitalWrite(MAIN, HIGH);
    }

  } else {
    Serial.println("Off");
    digitalWrite(PRE, LOW);
    digitalWrite(MAIN, LOW);
  }
  // Serial.println(digitalRead(BMS_IN));
}

void precharge() {
  // Turn pin 3 on and wait 2.5 sec
    Serial.println("Starting precharge");
    digitalWrite(PRE, HIGH);
    delay(2500); 

    // Turn pin 3 off, wait 0.5 sec
    digitalWrite(PRE, LOW);
    delay(500);
}