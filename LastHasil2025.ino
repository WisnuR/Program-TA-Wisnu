// LCD 1602 ------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);  //0x3f.0x27
// LCD 1602 END---------------------------

//STEPPER--------------------------------------------------------------------------------
byte limit = 13;
byte stepPin = 2;
byte dirPin = 3;

int step_turun = 5000;  //auto off
int step_naik = 2000;
int speed_lift = 1800;

void lift(bool z, long step, int speed) {
  for (long x = 0; x < step; x++) {
    // Serial.println(x);
    if (z == HIGH && digitalRead(limit) == LOW) {
      x = step;
    }
    digitalWrite(dirPin, z);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
  }
}
//STEPPER END----------------------------------------------------------------------------

//SERVO----------------------------------------------------------------------------------
#include <Servo.h>
byte servo_pin = 45;
Servo my_servo;
byte derajat_buka = 20;
byte derajat_tutup = 45;

void gripper_buka() {
  for (byte x = derajat_tutup; x > derajat_buka; x--) {  // gripper buka
    my_servo.write(x);
    // Serial.println(x);
    delay(50);
  }
}

void gripper_tutup() {
  for (byte x = derajat_buka; x < derajat_tutup; x++) {  // gripper tutup
    my_servo.write(x);
    // Serial.println(x);
    delay(50);
  }
}
//SERVO END-------------------------------------------------------------------------------

//SENSOR GARIS---------------------------------------------------------------------------
byte s1_pin = A0;
byte s2_pin = A1;
byte s3_pin = A2;
byte s4_pin = A3;
byte s5_pin = A4;
byte s6_pin = A5;
byte s7_pin = A6;
byte s8_pin = A7;

int analog_1, analog_2, analog_3, analog_4, analog_5, analog_6, analog_7, analog_8;
String sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6, sensor_7, sensor_8;
int treshold = 850;  // nilai analog > dari treshold = hitam
byte counter_line = 0;
bool flag = 0;

void reset_sensor() {
  analogRead(s1_pin);
  analogRead(s2_pin);
  analogRead(s3_pin);
  analogRead(s4_pin);
  analogRead(s5_pin);
  analogRead(s6_pin);
  analogRead(s7_pin);
  analogRead(s8_pin);
}
void display_sensor() {
  // SENSOR 1-4 = KIRI, SENSOR 4-8 = KANAN

  // Serial.println();
  // Serial.println(String() + "Sensor 1 = " + analogRead(s1_pin));
  // Serial.println(String() + "Sensor 2 = " + analogRead(s2_pin));
  // Serial.println(String() + "Sensor 3 = " + analogRead(s3_pin));
  // Serial.println(String() + "Sensor 4 = " + analogRead(s4_pin));
  // Serial.println(String() + "Sensor 5 = " + analogRead(s5_pin));
  // Serial.println(String() + "Sensor 6 = " + analogRead(s6_pin));
  // Serial.println(String() + "Sensor 7 = " + analogRead(s7_pin));
  // Serial.println(String() + "Sensor 8 = " + analogRead(s8_pin));

  if (analogRead(s1_pin) > treshold) {
    lcd.setCursor(0, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("0");
  }

  if (analogRead(s2_pin) > treshold) {
    lcd.setCursor(2, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(2, 0);
    lcd.print("0");
  }

  if (analogRead(s3_pin) > treshold) {
    lcd.setCursor(4, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(4, 0);
    lcd.print("0");
  }

  if (analogRead(s4_pin) > treshold) {
    lcd.setCursor(6, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(6, 0);
    lcd.print("0");
  }

  if (analogRead(s5_pin) > treshold) {
    lcd.setCursor(8, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(8, 0);
    lcd.print("0");
  }

  if (analogRead(s6_pin) > treshold) {
    lcd.setCursor(10, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(10, 0);
    lcd.print("0");
  }

  if (analogRead(s7_pin) > treshold) {
    lcd.setCursor(12, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(12, 0);
    lcd.print("0");
  }

  if (analogRead(s8_pin) > treshold) {
    lcd.setCursor(14, 0);
    lcd.print("1");
  } else {
    lcd.setCursor(14, 0);
    lcd.print("0");
  }

  // lcd.setCursor(0, 1);
  // lcd.print(analogRead(s1_pin));
  // lcd.print("   ");
}

bool boost_speed = true;
// 1= sensor paling kiri(sensor 1) untuk hitung garis, 2 = sensor paling kiri dan kanan (sensor 1 dan 8), 3 = sensor paling kanan (sensor 8)
byte mode_counter = 2;

// Tambahkan di luar fungsi (global)
bool is_cross_detected = false;
unsigned long last_cross_time = 0;
const unsigned long cross_debounce_time = 1000; // waktu tunggu agar tidak double (dalam ms)

void line_follower(int speed) {
  display_sensor();
  Serial.println("LF Mode");

  if (boost_speed == true) {
    speed += 20;
  }

  // === Navigasi dasar ===
  if ((analogRead(s1_pin) > treshold || analogRead(s2_pin) > treshold || analogRead(s3_pin) > treshold || analogRead(s4_pin) > treshold) &&
      analogRead(s5_pin) < treshold && analogRead(s6_pin) < treshold && analogRead(s7_pin) < treshold && analogRead(s8_pin) < treshold) {
    turn_left(speed);
  } else if (analogRead(s1_pin) < treshold && analogRead(s2_pin) < treshold && analogRead(s3_pin) < treshold && analogRead(s4_pin) < treshold &&
             (analogRead(s5_pin) > treshold || analogRead(s6_pin) > treshold || analogRead(s7_pin) > treshold || analogRead(s8_pin) > treshold)) {
    turn_right(speed);
  } else {
    forward(speed, speed);
    flag = 1;
  }

  // === Deteksi Perempatan ===
  bool all_black = 
    analogRead(s1_pin) > treshold &&
    analogRead(s2_pin) > treshold &&
    analogRead(s3_pin) > treshold &&
    analogRead(s4_pin) > treshold &&
    analogRead(s5_pin) > treshold &&
    analogRead(s6_pin) > treshold &&
    analogRead(s7_pin) > treshold &&
    analogRead(s8_pin) > treshold;

  // === Deteksi Pertigaan KIRI ===
  bool t_intersection_left =
    analogRead(s1_pin) > treshold &&
    analogRead(s2_pin) > treshold &&
    analogRead(s3_pin) > treshold &&
    analogRead(s4_pin) > treshold &&
    analogRead(s5_pin) < treshold &&
    analogRead(s6_pin) < treshold &&
    analogRead(s7_pin) < treshold &&
    analogRead(s8_pin) < treshold;

  // === Deteksi Pertigaan KANAN ===
  bool t_intersection_right =
    analogRead(s1_pin) < treshold &&
    analogRead(s2_pin) < treshold &&
    analogRead(s3_pin) < treshold &&
    analogRead(s4_pin) < treshold &&
    analogRead(s5_pin) > treshold &&
    analogRead(s6_pin) > treshold &&
    analogRead(s7_pin) > treshold &&
    analogRead(s8_pin) > treshold;

  unsigned long current_time = millis();

  // === Tambah counter jika perempatan atau pertigaan terdeteksi
  if ((all_black || t_intersection_left || t_intersection_right) &&
      !is_cross_detected && (current_time - last_cross_time > cross_debounce_time)) {
    
    counter_line++;
    lcd.noBacklight();
    forward(speed, speed);
    delay(100);  // biar benar-benar lewat sedikit
    lcd.backlight();
    is_cross_detected = true;
    last_cross_time = current_time;
  }

  // === Reset status deteksi ===
  if (!all_black && !t_intersection_left && !t_intersection_right && is_cross_detected) {
    is_cross_detected = false;
  }

  boost_speed = false;

  lcd.setCursor(8, 1);
  lcd.print(counter_line);
  lcd.print(" ");
}


//SENSOR GARIS END ------------------------------------------------------------------------

// GM65 SCANNER----------------------------
byte pin_on_off_scanner = 14;
void on_off_scanner() {
  digitalWrite(pin_on_off_scanner, HIGH);
  delay(500);
  digitalWrite(pin_on_off_scanner, LOW);
  delay(50);
}

String Scanner = "";
void SCANNER() {
  while (Scanner == "") {
    while (Serial1.available() > 0) {
      Scanner = Serial1.readString();
      Scanner.trim();
      Serial.println(Scanner);
      stop();  // dc motor stop
    }
  }
}
// GM65 SCANNER END------------------------

// DRIVER MOTOR ---------------------------
//DRIVER MOTOR DEPAN
byte FL_IN1 = 7;
byte FL_IN2 = 6;
byte FR_IN3 = 5;
byte FR_IN4 = 4;

//DRIVER MOTOR BELAKANG
byte RL_IN4 = 11;
byte RL_IN3 = 10;
byte RR_IN2 = 9;
byte RR_IN1 = 8;

void forward(int left_speed, int right_speed) {
  analogWrite(FL_IN1, left_speed);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FR_IN3, right_speed);
  digitalWrite(FR_IN4, LOW);

  analogWrite(RL_IN4, left_speed);
  digitalWrite(RL_IN3, LOW);
  analogWrite(RR_IN2, right_speed);
  digitalWrite(RR_IN1, LOW);
}

void backward(int left_speed, int right_speed) {
  digitalWrite(FL_IN1, LOW);
  analogWrite(FL_IN2, left_speed);
  digitalWrite(FR_IN3, LOW);
  analogWrite(FR_IN4, right_speed);

  digitalWrite(RL_IN4, LOW);
  analogWrite(RL_IN3, left_speed);
  digitalWrite(RR_IN2, LOW);
  analogWrite(RR_IN1, right_speed);
}

void turn_left(int speed) {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  digitalWrite(RL_IN3, LOW);
  analogWrite(RR_IN2, speed);
  digitalWrite(RR_IN1, LOW);
}

void turn_right(int speed) {
  analogWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  analogWrite(RL_IN4, speed);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
}

void spin_left(int speed) {
  digitalWrite(FL_IN1, LOW);
  analogWrite(FL_IN2, speed);
  analogWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  analogWrite(RL_IN3, speed);
  analogWrite(RR_IN2, speed);
  digitalWrite(RR_IN1, LOW);
}

void spin_right(int speed) {
  analogWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  analogWrite(FR_IN4, speed);

  analogWrite(RL_IN4, speed);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  analogWrite(RR_IN1, speed);
}

void slide_left(int left_speed, int right_speed) {
  digitalWrite(FL_IN1, LOW);
  analogWrite(FL_IN2, left_speed);
  analogWrite(FR_IN3, right_speed);
  digitalWrite(FR_IN4, LOW);

  analogWrite(RL_IN4, left_speed);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  analogWrite(RR_IN1, right_speed);
}

void slide_right(int left_speed, int right_speed) {
  analogWrite(FL_IN1, left_speed);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  analogWrite(FR_IN4, right_speed);

  digitalWrite(RL_IN4, LOW);
  analogWrite(RL_IN3, left_speed);
  analogWrite(RR_IN2, right_speed);
  digitalWrite(RR_IN1, LOW);
}

void diagonal_front_left(int left_speed, int right_speed) {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FR_IN3, right_speed);
  digitalWrite(FR_IN4, LOW);

  analogWrite(RL_IN4, left_speed);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
}

void diagonal_front_right(int left_speed, int right_speed) {
  analogWrite(FL_IN1, left_speed);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  digitalWrite(RL_IN3, LOW);
  analogWrite(RR_IN2, right_speed);
  digitalWrite(RR_IN1, LOW);
}

void diagonal_rear_left(int left_speed, int right_speed) {
  digitalWrite(FL_IN1, LOW);
  analogWrite(FL_IN2, left_speed);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  analogWrite(RR_IN1, right_speed);
}

void diagonal_rear_right(int left_speed, int right_speed) {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  analogWrite(FR_IN4, right_speed);

  digitalWrite(RL_IN4, LOW);
  analogWrite(RL_IN3, left_speed);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
}

void stop() {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
}
// DRIVER MOTOR END -----------------------

void setup() {
  Serial.begin(9600);
  // LCD 1602 ------------------------------
  //lcd.init();
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("MOBILE");
  lcd.setCursor(0, 1);
  lcd.print("ROBOT");
  delay(1000);
  lcd.clear();
  // LCD 1602 END --------------------------
  // STEPPER ----------------------------------------------------------------------------------
  pinMode(limit, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  lift(HIGH, step_turun, speed_lift);  //high = turun, step, speed
  // STEPPER END -------------------------------------------------------------------------------

  // SERVO----------------------------------------------------------------------------------
  my_servo.attach(servo_pin);
  my_servo.write(derajat_buka);
  // SERVO END -----------------------------------------------------------------------------

  // GM65 SCANNER----------------------------
  Serial1.begin(9600);
  Serial1.setTimeout(100);
  pinMode(pin_on_off_scanner, OUTPUT);
  on_off_scanner();
  // GM65 SCANNER END------------------------

  // DRIVER MOTOR ---------------------------
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  pinMode(RL_IN4, OUTPUT);
  pinMode(RL_IN3, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);

  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  digitalWrite(RL_IN4, LOW);
  digitalWrite(RL_IN3, LOW);
  digitalWrite(RR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
  // DRIVER MOTOR END -----------------------
}

String barcode_1 = "Paket 2"; //code star 2 done
String barcode_2 = "Paket 3"; //code star 3 kendala pembacaan lurus
String barcode_3 = "Paket 1"; //code star 1 done
String barcode_4 = "Paket 4";
String barcode_5 = "Paket 5";

byte flag_2 = 0;
byte flag_3 = 0;

bool gerakan_barcode_1 = false;
bool gerakan_barcode_2 = false;
bool gerakan_barcode_3 = false;
bool gerakan_barcode_4 = false;
bool gerakan_barcode_5 = false;

void loop() {
  int speed_LF = 80;
  int speed_move = 100;

  while (flag_2 == 0) {
    for (int x = 0; x <= 3; x++) {
      Serial1.flush();
      Scanner = "";
      delay(400);
    }
    // lcd.setCursor(0, 1);
    // lcd.print(Scanner);
    // lcd.print("   ");

    counter_line = 0;
    lcd.setCursor(14, 1);
    lcd.print("A1");
    mode_counter = 2;
    while (counter_line == 0) {
      line_follower(speed_LF);
    }
    while (counter_line == 1) {
      stop();
      forward(speed_move, speed_move);
      delay(50);
      stop();
      Scanner = "";
      delay(100);
      on_off_scanner();
      SCANNER();
      // Serial.println(Scanner);

      lcd.setCursor(0, 1);
      lcd.print(Scanner);
      lcd.print("   ");
      delay(100);

      // scan gerakan 1---------------------------------------------------
      if (Scanner == barcode_1) {
        on_off_scanner();
        lcd.setCursor(14, 1);
        lcd.print("S1");
        while (analogRead(s1_pin) < treshold || analogRead(s8_pin) < treshold) {
          forward(speed_move, speed_move);
        }
        stop();
        delay(100);

        gripper_tutup();
        lift(LOW, step_naik, speed_lift);  //low = naik, step, speed
        stop();
        delay(100);

        reset_sensor();
        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        delay(2700);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        stop();
        delay(100);

        counter_line = -1;
        flag_2 = 1;
        flag_3 = 0;

        gerakan_barcode_1 = true;
        gerakan_barcode_2 = false;
        gerakan_barcode_3 = false;
        gerakan_barcode_4 = false;
        gerakan_barcode_5 = false;
        delay(1000);
      }
      // end scan gerakan 1-----------------------------------------------

     // scan gerakan 2---------------------------------------------------
      if (Scanner == barcode_2) {
        on_off_scanner();
        lcd.setCursor(14, 1);
        lcd.print("S2");
        while (analogRead(s1_pin) < treshold || analogRead(s8_pin) < treshold) {
          forward(speed_move, speed_move);
        }
        stop();
        delay(100);

        gripper_tutup();
        lift(LOW, step_naik, speed_lift);  //low = naik, step, speed
        stop();
        delay(100);

        reset_sensor();
        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        delay(2700);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        stop();
        delay(100);

        counter_line = -1;
        flag_2 = 1;
        flag_3 = 0;

        gerakan_barcode_1 = false;
        gerakan_barcode_2 = true;
        gerakan_barcode_3 = false;
        gerakan_barcode_4 = false;
        gerakan_barcode_5 = false;
        delay(1000);
      }
      // end scan gerakan 2-----------------------------------------------

      // scan gerakan 3---------------------------------------------------
      if (Scanner == barcode_3) {
        on_off_scanner();
        lcd.setCursor(14, 1);
        lcd.print("S3");
        while (analogRead(s1_pin) < treshold || analogRead(s8_pin) < treshold) {
          forward(speed_move, speed_move);
        }
        stop();
        delay(100);

        gripper_tutup();
        lift(LOW, step_naik, speed_lift);  //low = naik, step, speed
        stop();
        delay(100);

        reset_sensor();
        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        delay(2500);

        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        stop();
        delay(100);

        counter_line = -1;
        flag_2 = 1;
        flag_3 = 0;

        gerakan_barcode_1 = false;
        gerakan_barcode_2 = false;
        gerakan_barcode_3 = true;
        gerakan_barcode_4 = false;
        gerakan_barcode_5 = false;
        delay(1000);
      }
      // end scan gerakan 3-----------------------------------------------

      // scan gerakan 4---------------------------------------------------
      if (Scanner == barcode_4) {
        on_off_scanner();
        lcd.setCursor(14, 1);
        lcd.print("S4");
        while (analogRead(s1_pin) < treshold || analogRead(s8_pin) < treshold) {
          forward(speed_move, speed_move);
        }
        stop();
        delay(100);

        gripper_tutup();
        lift(LOW, step_naik, speed_lift);  //low = naik, step, speed
        stop();
        delay(100);

        reset_sensor();
        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        delay(2700);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        stop();
        delay(100);

        counter_line = -1;
        flag_2 = 1;
        flag_3 = 0;

        gerakan_barcode_1 = false;
        gerakan_barcode_2 = false;
        gerakan_barcode_3 = false;
        gerakan_barcode_4 = true;
        gerakan_barcode_5 = false;
        delay(1000);
      }
      // end scan gerakan 4-----------------------------------------------

      // scan gerakan 5---------------------------------------------------
      if (Scanner == barcode_5) {
        on_off_scanner();
        lcd.setCursor(14, 1);
        lcd.print("S5");
        while (analogRead(s1_pin) < treshold || analogRead(s8_pin) < treshold) {
          forward(speed_move, speed_move);
        }
        stop();
        delay(100);

        gripper_tutup();
        lift(LOW, step_naik, speed_lift);  //low = naik, step, speed
        stop();
        delay(100);

        reset_sensor();
        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        delay(2500);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        stop();
        delay(100);

        counter_line = -1;
        flag_2 = 1;
        flag_3 = 0;

        gerakan_barcode_1 = false;
        gerakan_barcode_2 = false;
        gerakan_barcode_3 = false;
        gerakan_barcode_4 = false;
        gerakan_barcode_5 = true;
        delay(1000);
      }
      // end scan gerakan 5-----------------------------------------------
    }
  }

  // gerakan 1 start ke 2-----------------------------------------------
while (gerakan_barcode_1) {
    lcd.setCursor(14, 1);
    lcd.print("G1");
    while (flag_3 == 0) {
        counter_line = 0;
        boost_speed = true;
        mode_counter = 2;
        while (counter_line >= -1 && counter_line < 2) {
            line_follower(speed_LF);
        }
        while (counter_line == 2) {
            stop();
            forward(speed_move, speed_move);
            delay(400);
            stop();
            delay(100);
            slide_left(speed_move, speed_move);
            delay(4000); //try n error sampai lewat garis 2
            // while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
            //     slide_left(speed_move, speed_move);
            // }
            slide_left(speed_move, speed_move);
            while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
                slide_left(speed_move, speed_move);
            }
            stop();
            delay(100);
            counter_line = -1;
            delay(100);
            flag_3 = 1;
            delay(1000);
        }
    }
    while (flag_3 == 1) {
        stop();
        delay(100);
        counter_line = 0;
        boost_speed = true;
        mode_counter = 3;
        while (counter_line > -1 && counter_line < 3) {
            line_follower(speed_LF);
        }
        while (counter_line == 3) {
            forward(speed_move, speed_move);
            delay(800);
            stop();
            delay(100);
            lift(HIGH, step_turun, speed_lift);
            gripper_buka();
            backward(speed_move, speed_move);
            delay(1000);
            spin_right(speed_move);
            delay(2700);
            while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
                spin_right(speed_move);
            }
            reset_sensor();
            stop();
            delay(500);
            counter_line = -1;
            delay(100);
            flag_3 = 2;
            delay(1000);
        }
    }
    while (flag_3 == 2) {
    stop();
    delay(100);
    counter_line = 0;
    boost_speed = true;
    mode_counter = 1;
    
    // Mengikuti garis sampai mencapai counter_line = 4
    while (counter_line > -1 && counter_line < 3) {
        line_follower(speed_LF);
    }

    // Ketika counter_line == 4, lakukan gerakan tambahan
    while (counter_line == 3) {
        forward(speed_move, speed_move);
        delay(400);
        stop();
        delay(500);
            slide_left(speed_move, speed_move);
            delay(4000); //try n error sampai lewat garis 2
            // while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
            //     slide_left(speed_move, speed_move);
            // }
            slide_left(speed_move, speed_move);
            while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
                slide_left(speed_move, speed_move);
            }
        stop();
        delay(100);
        counter_line = -1;
        delay(100);
        flag_3 = 3;
        Scanner = "";
        delay(1000);
    }
}
    while (flag_3 == 3) {
        stop();
        delay(100);
        counter_line = -1;
        flag_2 = 0;
        gerakan_barcode_1 = false;
        flag_3 = -1;
        Scanner = "";
        lcd.clear();
        delay(1000);
    }

}
  // end gerakan 1-----------------------------------------------

  // gerakan 2 star ke 3-----------------------------------------------
   while (gerakan_barcode_2) {
    lcd.setCursor(14, 1);
    lcd.print("G2");
    while (flag_3 == 0) {
      counter_line = 0;
      boost_speed = true;
      mode_counter = 2;
      while (counter_line >= -1 && counter_line < 2) {
        line_follower(speed_LF);
      }
      while (counter_line == 2) {
        stop();
        forward(speed_move, speed_move);
        delay(500);
        stop();
        delay(100);
        slide_left(speed_move, speed_move);
        delay(300);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          slide_left(speed_move, speed_move);  //speed kiri, speed kanan
        }
        stop();
        delay(100);

        counter_line = -1;
        delay(100);
        flag_3 = 1;
        delay(1000);
      }
    }
    while (flag_3 == 1) {
      stop();
      delay(100);
      counter_line = 0;
      boost_speed = true;
      mode_counter = 2;  // 2 = sensor paling kiri dan kanan (sensor 1 dan 8) untuk hitung garis
      while (counter_line > -1 && counter_line < 3) {
        line_follower(speed_LF);
      }
      while (counter_line == 3) {
        forward(speed_move, speed_move);
        delay(800);
        stop();
        delay(100);
        lift(HIGH, step_turun, speed_lift);  //HIGH = turun, step, speed
        gripper_buka();

        backward(speed_move, speed_move);
        delay(1000);
        spin_right(speed_move);
        reset_sensor();
        delay(1500);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        reset_sensor();
        stop();
        delay(100);

        counter_line = -1;  //Safe point
        delay(100);
        flag_3 = 2;
        delay(1000);
      }
    }
    while (flag_3 == 2) {
      stop();
      delay(100);
      counter_line = 0;
      boost_speed = true;
      mode_counter = 1;  // 1 = sensor paling kiri (sensor 1) untuk hitung garis
      while (counter_line > -1 && counter_line < 3) {
        line_follower(speed_LF);
      }
      // forward(speed_move, speed_move);
      // delay(5000); //try n error
      counter_line =3;
      while (counter_line == 3) {
        forward(speed_move, speed_move);
        delay(400);
        stop();
        delay(100);
        slide_left(speed_move, speed_move);
        delay(200);
        while (analogRead(s4_pin) < treshold || analogRead(s5_pin) < treshold) {
          slide_left(speed_move, speed_move);  //speed kiri, speed kanan
        }
        stop();
        delay(100);
        counter_line = -1;
        flag_3 = 3;
        Scanner = "";
        delay(1000);
      }
    }
    while (flag_3 == 3) {
      stop();
      delay(100);
      counter_line = -1;
      flag_2 = 0;
      gerakan_barcode_2 = false;
      flag_3 = -1;
      Scanner = "";
      lcd.clear();
      delay(1000);
    }
    Scanner = "";
    delay(100);
  }
  // end gerakan 2-----------------------------------------------

  // gerakan 3 start 1-----------------------------------------------
while (gerakan_barcode_3) {
    lcd.setCursor(14, 1);
    lcd.print("G3");
    while (flag_3 == 0) {
      // lcd.setCursor(14, 1);
      // lcd.print(flag_2);
      // lcd.print(" ");
      counter_line = 0;
      boost_speed = true;
      mode_counter = 2;
      while (counter_line >= -1 && counter_line < 5) {
        line_follower(speed_LF);
      }
      while (counter_line == 5) {
        stop();
        forward(speed_move, speed_move);
        delay(500);
        stop();
        delay(100);
        lift(HIGH, step_turun, speed_lift);  //HIGH = turun, step, speed
        gripper_buka();

        backward(speed_move, speed_move);
        delay(100);
        spin_right(speed_move);
        reset_sensor();
        delay(1500);

        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        reset_sensor();
        stop();
        delay(100);

        counter_line = -1;
        delay(100);
        flag_3 = 1;
        delay(1000);
      }
    }

    while (flag_3 == 1) {
      // lcd.setCursor(14, 1);
      // lcd.print(flag_2);
      // lcd.print(" ");

      counter_line = 0;
      boost_speed = true;
      mode_counter = 2;
      while (counter_line > -1 && counter_line < 3) {
        line_follower(speed_LF);
      }
      stop();
      forward(speed_move, speed_move);
      delay(300);
      while (counter_line == 3) {
        stop();
        delay(100);
        counter_line = -1;
        flag_2 = 0;
        gerakan_barcode_3 = false;
        flag_3 = -1;
        lcd.clear();
        delay(1000);
      }
    }
    Scanner = "";
    delay(100);
  }

 // gerakan 4-----------------------------------------------
while (gerakan_barcode_4) {
  lcd.setCursor(14, 1);
  lcd.print("G4");

  // Tahap 1: Pengambilan paket
  while (flag_3 == 0) {
    counter_line = 0;
    boost_speed = true;
    mode_counter = 2;  // Sensor kiri dan kanan untuk hitung garis
    while (counter_line >= -1 && counter_line < 3) {
      line_follower(speed_LF);
    }
    while (counter_line == 3) {
      stop();
      forward(speed_move, speed_move);
      delay(400);
      stop();
      delay(100);
      slide_right(speed_move, speed_move);
      delay(2000);
      while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
        slide_right(speed_move, speed_move);
      }
      stop();
      delay(100);
      counter_line = -1;
      delay(100);
      flag_3 = 1;
      delay(1000);
    }
  }

  // Tahap 2: Pengantaran paket dan turunkan paket
  while (flag_3 == 1) {
    stop();
    delay(100);
    counter_line = 0;
    boost_speed = true;
    mode_counter = 2;
    while (counter_line > -1 && counter_line < 2) {
      line_follower(speed_LF);
    }
    while (counter_line == 2) {
      forward(speed_move, speed_move);
      delay(800);
      stop();
      delay(100);
      lift(HIGH, step_turun, speed_lift);  // Turunkan paket
      gripper_buka();
      backward(speed_move, speed_move);
      delay(1000);
      spin_right(speed_move);
      reset_sensor();
      delay(100);
      while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
        spin_right(speed_move);
      }
      reset_sensor();
      stop();
      delay(500);
      counter_line = -1;
      delay(100);
      flag_3 = 2;
      delay(1000);
    }
  }

  // Tahap 3: Belok kanan setelah garis ketiga
  while (flag_3 == 2) {
    stop();
    delay(100);
    counter_line = 0;
    boost_speed = true;
    mode_counter = 1;  // Sensor paling kiri untuk hitung garis
    while (counter_line > -1 && counter_line < 3) {
      line_follower(speed_LF);
    }
    while (counter_line == 3) {
      forward(speed_move, speed_move);
      delay(400);
      stop();
      delay(100);
      slide_right(speed_move, speed_move);
      delay(2000);
      slide_left(speed_move, speed_move);
      while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
        slide_right(speed_move, speed_move);
      }
      stop();
      delay(100);
      counter_line = -1;
      delay(100);

      flag_3 = 3;
      Scanner = "";
      delay(1000);
    }
  }

  // Tahap 4: Selesai
  while (flag_3 == 3) {
    stop();
    delay(100);
    counter_line = -1;
    flag_2 = 0;
    gerakan_barcode_4 = false;
    flag_3 = -1;
    Scanner = "";
    lcd.clear();
    delay(1000);
  }

  Scanner = "";
  delay(100);
}
  // end gerakan 4-----------------------------------------------

// terus nya tambah
// belok kanan kurang
// gerakan 5-----------------------------------------------
  // gerakan 5-----------------------------------------------
  while (gerakan_barcode_5) {
    lcd.setCursor(14, 1);
    lcd.print("G5");
    while (counter_line >= -1 && counter_line < 2) {
    line_follower(speed_LF);
    }
    while (flag_3 == 0) {
      counter_line = 0;
      boost_speed = true;
      mode_counter = 2;  // 2 = sensor paling kiri dan kanan (sensor 1 dan 8) untuk hitung garis
      while (counter_line >= -1 && counter_line < 3) {
        line_follower(speed_LF);
      }
      while (counter_line == 3) {
        stop();
        forward(speed_move, speed_move);
        delay(500);
        stop();
        delay(100);
        slide_right(speed_move, speed_move);
        delay(4000);
        slide_right(speed_move, speed_move);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          slide_right(speed_move, speed_move);  //speed kiri, speed kanan
        }
        stop();
        delay(100);

        counter_line = -1;
        delay(100);
        flag_3 = 1;
        delay(1000);
      }
    }
    while (flag_3 == 1) {
      stop();
      delay(100);
      counter_line = 0;
      boost_speed = true;
      mode_counter = 1;  // 1 = sensor paling kiri  (sensor 1) untuk hitung garis
      while (counter_line > -1 && counter_line < 2) {
        line_follower(speed_LF);
      }
      while (counter_line == 2) {
        forward(speed_move, speed_move);
        delay(800);
        stop();
        delay(100);
        lift(HIGH, step_turun, speed_lift);  //HIGH = turun, step, speed
        gripper_buka();

        backward(speed_move, speed_move);
        delay(1000);
        spin_right(speed_move);
        reset_sensor();
        delay(100);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
          spin_right(speed_move);
        }
        reset_sensor();
        stop();
        delay(100);

        counter_line = -1;
        delay(100);
        flag_3 = 2;
        delay(1000);
      }
    }
    while (flag_3 == 2) {
      stop();
      delay(100);
      counter_line = 0;
      boost_speed = true;
      mode_counter = 1;  // 1 = sensor paling kiri (sensor 1) untuk hitung garis
      while (counter_line > -1 && counter_line < 3) {
        line_follower(speed_LF);
      }
      // forward(speed_move, speed_move);
      // delay(5500); //try n error
      counter_line =3;
      while (counter_line == 3) {
        forward(speed_move, speed_move);
        delay(400);
        stop();
        delay(100);
        slide_right(speed_move, speed_move);
        delay(4000);
        while (analogRead(s5_pin) < treshold || analogRead(s6_pin) < treshold) {
        slide_right(speed_move, speed_move);  //speed kiri, speed kanan
        }
        stop();
        delay(100);

        counter_line = -1;
        delay(100);
        flag_3 = 3;
        Scanner = "";
        delay(1000);

        flag_3 = 3;
        Scanner = "";
        delay(1000);
      }
    }
    while (flag_3 == 3) {
      stop();
      delay(100);
      counter_line = -1;
      flag_2 = 0;
      gerakan_barcode_5 = false;
      flag_3 = -1;
      Scanner = "";
      lcd.clear();
      delay(1000);
    }
    Scanner = "";
    delay(100);
  }
}
