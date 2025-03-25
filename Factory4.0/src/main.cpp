#include <Arduino.h>
#include <pio_encoder.h>
#include "autopid.h"

#define PIN_IN1_M1 2
#define PIN_IN2_M1 3
#define PIN_ENC_A_M1 4
#define PIN_ENC_B_M1 5

#define PIN_IN3_M2
#define PIN_IN4_M2
#define PIN_ENC_A_M2
#define PIN_ENC_B_M2

PioEncoder encoder(PIN_ENC_A_M1);

double kc = 0.0, tc = 1e-3f, kp = 1.0, ki = 0.0, kd = 0.00;

double get_rpm() {
    static double rpm = 0;
    static int tick = 0;
    const double ppr = 823.1;
    const int dt = 10000;

    if (time_us_32() - tick >= dt) {
        rpm = (encoder.getCount() * 60e6) / (ppr * dt);
        encoder.reset();
        tick = time_us_32();
    }

    return rpm;
}

double process(double pwm) {    
    if (pwm > 0) {
        analogWrite(PIN_IN1_M1, abs(pwm));
        analogWrite(PIN_IN2_M1, 0);
    } else {
        analogWrite(PIN_IN1_M1, 0);
        analogWrite(PIN_IN2_M1, abs(pwm));    
    }
    return get_rpm();
}

void setup() {
    Serial.begin(9600);

    pinMode(PIN_IN1_M1, OUTPUT);
    pinMode(PIN_IN2_M1, OUTPUT);

    encoder.begin();

    delay(4000);
    
    autopid(kc, tc, kp, ki, kd, 100, process);
}

void loop() {
    static double setpoint = -100, measured = get_rpm();
    double pwm = pid(setpoint, measured, kp, ki, kd, 10);
    
    delay(10);
    
    measured = process(pwm);
    Serial.println("Rotations per Minute:" + String(get_rpm()));
}