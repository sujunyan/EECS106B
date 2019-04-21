int left = 0;
int right = 0;

const int LEFT_FINGER_PIN = 5;
const int RIGHT_FINGER_PIN = 7;
const int LEFT_PRESSURE_PIN = A0;
const int RIGHT_PRESSURE_PIN = A1;
const int LEFT_FLEX_PIN = A2;
const int RIGHT_FLEX_PIN = A3;

void setup() {
  // put your setup code here, to run once:

  pinMode(LEFT_FINGER_PIN, OUTPUT);
  pinMode(RIGHT_FINGER_PIN, OUTPUT);
  pinMode(LEFT_FLEX_PIN, INPUT);
  pinMode(RIGHT_FLEX_PIN, INPUT);
  Serial.begin(9600);

}

void loop() {
  // send command
  // This is super hacky.....I hate arduino - Chris
  if (Serial.available() > 1) {
    left = Serial.parseInt();
    right = Serial.parseInt();
    //Serial.println(String(left)+","+String(right));
    left = min(left, 200);
    right = min(right, 200);
    analogWrite(LEFT_FINGER_PIN, left);
    analogWrite(RIGHT_FINGER_PIN, right);
  }

  // read sensors
  float curr_time    = millis()/1000.0;
  int left_pressure  = analogRead(LEFT_PRESSURE_PIN);
  int right_pressure = analogRead(RIGHT_PRESSURE_PIN);
  int left_flex  = analogRead(LEFT_FLEX_PIN);
  int right_flex = analogRead(RIGHT_FLEX_PIN);

  Serial.println(String(curr_time)+","+String(left)+","+String(right)+","+String(left_pressure)+","+String(right_pressure)+","+String(left_flex)+","+String(right_flex));
  delay(10);
}
