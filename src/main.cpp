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

double kc = 0.0, kp = 0.0, ki = 0.0, kd = 0.00, tc = 1.0 / 1000.0;

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
        analogWrite(PIN_IN2_M1, pwm);    
    }
    return get_rpm();
}

double pid(double setpoint, double measured, double kp, double ki, double kd, double dt) {
    static double integral = 0, prev_error = 0;
    double error = setpoint - measured;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}

void setup() {
    Serial.begin(9600);

    pinMode(PIN_IN1_M1, OUTPUT);
    pinMode(PIN_IN2_M1, OUTPUT);

    encoder.begin();

    delay(4000);
    
    findCriticalValues(&kc, &tc, process);
    calculatePID(kc, tc, &kp, &ki, &kd);
}

void loop() {
    double setpoint = -100, measured = get_rpm();
    double pwm = pid(setpoint, measured, kp, ki, kd, 10);

    if (pwm > 0) {
        analogWrite(PIN_IN1_M1, pwm);
        analogWrite(PIN_IN2_M1, 0);
    } else {
        analogWrite(PIN_IN1_M1, 0);
        analogWrite(PIN_IN2_M1, abs(pwm));    
    }
    
    delay(10);
    Serial.println("Rotations per Minute:" + String(get_rpm()));
}