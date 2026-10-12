int lichtR = 39;
int lichtM = 34;
int lichtL = 35;

void setup() {
  Serial.begin(115200);
  pinMode(lichtR, INPUT);
  pinMode(lichtM, INPUT);
  pinMode(lichtL, INPUT);
}

void loop() {
  analogWrite(21, 110);
  delay(10);
  int weiß = (analogRead(lichtR) + analogRead(lichtM) + analogRead(lichtL)) / 3;
  while(true){
    Serial.println(analogRead(lichtR));
    Serial.println(analogRead(lichtM));
    Serial.println(analogRead(lichtL));
    bool lR = analogRead(lichtR) > weiß;
    bool lM = analogRead(lichtM) > weiß;
    bool lL = analogRead(lichtL) > weiß;
    if (lR && lM && lL) Serial.println("freie fahrt");
    else if (!lR && !lM && !lL) Serial.println("stopp");
    else if (!lR && lM && lL) Serial.println("rechtskurve");
    else if (lR && !lM && lL) Serial.println("geradeaus");
    else if (lR && lM && !lL) Serial.println("linkskurve");
    else if (!lR && !lM && lL) Serial.println("rechtsecke");
    else if (!lR && lM && !lL) Serial.println("kreuzung");
    else if (lR && !lM && !lL) Serial.println("linksecke");
    delay(10);
  }
}
