#include <Arduino.h>
#include <Servo.h>

#define BUTT_STATIC 2
#define BUTT_SWEEP 4
#define BUTT_FOLLOW_FACE 6
#define BUTT_FOLLOW_BALL 8

#define LED_STATIC 3
#define LED_SWEEP 5
#define LED_FOLLOW_FACE 7
#define LED_FOLLOW_BALL 3

#define SERVO_PIN 10
#define DC_PIN1 11
#define DC_PIN2 12
#define DC_PINEN 13

#define FAN_STATIC 0
#define FAN_SWEEP 1
#define FAN_FOLLOW_FACE 2
#define FAN_FOLLOW_BALL 3

#define JOY_PIN_X A0
#define JOY_PIN_Y A1

byte state = FAN_STATIC;
Servo tilt;
unsigned long prev = 0;
double degree_per_ms = 0.05;
double tilt_position = 90.0;

int sweep_pos = 1000;
int sweep_increment = 1;

void drive_dc_forward() {
  digitalWrite(DC_PIN1, LOW);
  digitalWrite(DC_PIN2, HIGH);
}

void drive_dc_backward() {
  digitalWrite(DC_PIN1, HIGH);
  digitalWrite(DC_PIN2, LOW);
}

void stop_dc() {
  digitalWrite(DC_PIN1, HIGH);
  digitalWrite(DC_PIN2, HIGH);
}

short get_joy_x() {
  int diff = analogRead(JOY_PIN_X) - 512;

  if(abs(diff) > 100) return sign(diff);
  else return 0;
}

short get_joy_y() {
  int diff = analogRead(JOY_PIN_Y) - 512;

  if(abs(diff) > 100) return sign(diff);
  else return 0;
}

void clamp(int & val) {
  val = min(max(val, 10), 170);
}

void apply_dc_settings(String cmd) {
  if(cmd == "UP") {
    analogWrite(DC_PINEN, 100);
    drive_dc_forward();
  }
  else if(cmd == "UPP") {
    analogWrite(DC_PINEN, 150);
    drive_dc_forward();
  }
  else if(cmd == "UPPP") {
    analogWrite(DC_PINEN, 200);
    drive_dc_forward();
  }
  if(cmd == "DOWN") {
    analogWrite(DC_PINEN, 100);
    drive_dc_backward();
  }
  else if(cmd == "DOWNN") {
    analogWrite(DC_PINEN, 150);
    drive_dc_backward();
  }
  else if(cmd == "DOWNNN") {
    analogWrite(DC_PINEN, 200);
    drive_dc_backward();
  }
}

void apply_servo_settings(String cmd, double dt) {
  if(cmd == "UP") tilt_position += degree_per_ms * dt;
  else if(cmd == "UPP") tilt_position += 2.0 * degree_per_ms * dt;
  else if(cmd == "UPPP") tilt_position += 3.0 * degree_per_ms * dt;
  else if(cmd == "DOWN") tilt_position -= degree_per_ms * dt;
  else if(cmd == "DOWNN") tilt_position -= 2.0 * degree_per_ms * dt;
  else if(cmd == "DOWNNN") tilt_position -= 3.0 * degree_per_ms * dt;
  clamp(tilt_position);
  tilt.write(tilt_position)
}

void setup() {
  pinMode(BUTT_STATIC, INPUT_PULLUP);
  pinMode(BUTT_SWEEP, INPUT_PULLUP);
  pinMode(BUTT_FOLLOW_FACE, INPUT_PULLUP);
  pinMode(BUTT_FOLLOW_BALL, INPUT_PULLUP);

  pinMode(LED_STATIC, OUTPUT);
  pinMode(LED_SWEEP, OUTPUT);
  pinMode(LED_FOLLOW_FACE, OUTPUT);
  pinMode(LED_FOLLOW_BALL, OUTPUT);

  digitalWrite(LED_STATIC, LOW);
  digitalWrite(LED_SWEEP, LOW);
  digitalWrite(LED_FOLLOW_FACE, LOW);
  digitalWrite(LED_FOLLOW_BALL, LOW);

  tilt.attach(SERVO_PIN);
  tilt.write(tilt_position);

  pinMode(JOY_PIN_X, INPUT);
  pinMOde(JOY_PIN_Y, INPUT);

  pinMode(DC_PIN1, OUTPUT);
  pinMode(DC_PIN2, OUTPUT);
  pinMode(DC_PINEN, OUTPUT);
  digitalWrite(DC_PINEN, 100);

  Serial.begin(9600);
  prev = millis();
}

void loop() {
  int dt = millis() - prev;
  prev = millis();
  if(get_joy_x() || get_joy_y()) {
    state = FAN_STATIC;

    if(get_joy_x() == 1) drive_dc_forward();
    else if(get_joy_x() == -1) drive_dc_backward();

    if(get_joy_y() == 1) tilt_position += dt * degree_per_ms;
    else if(get_joy_y() == -1) tilt_position -= dt * degree_per_ms;
    clamp(tilt_position);
    tilt.write(tilt_position);
  }

  if(state == FAN_STATIC) {
    stop_dc();
  }
  else if(state == FAN_SWEEP) {
    if(sweep_pos >= 2000) {
      drive_dc_backward();
      sweep_increment = -1;
    }
    else if(sweep_pos <= 0) {
      drive_dc_forward();
      sweep_increment = 1;
    }
    sweep_pos = dt * sweep_increment;
  }
  else if(state == FAN_FOLLOW_BALL || state == FAN_FOLLOW_FACE) {
    while(Serial.available()) {
      data = Serial.read();
      msg.concat(data);
    }
    if(data == '\n' && msg.size()) {
      String fst = "", scnd = "";
      int pos = 0;
      while(msg[pos]!=' ')
        fst.concat(msg[pos++]);
      pos++;
      while(msg[pos]!='\n')
        scnd.concat(msg[pos++]);
      apply_dc_settings(fst);
      apply_servo_settings(scnd, dt);
    }
  }
}