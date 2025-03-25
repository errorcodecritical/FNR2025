#include <Arduino.h>

inline uint32_t tick() {
    return time_us_32();
}

// Function to determine critical gain (kc) and oscillation period (tc)
void autopid(double& kc, double& tc, double& kp, double& ki, double& kd, double setpoint, double(*process)(double)) {
    double output = 0, last_output = 0;
    double last_peak_time = tick(), first_peak_time = tick();
    int oscillation_count = 0;
    int first_peak_detected = 0;

    Serial.printf("Finding kc and tc...\n");

    // Wait for 5 oscillations
    while (oscillation_count < 5) {
        double control = kp * (setpoint - output);
        output = process(control);

        Serial.printf("rpm:%f, kp:%f\n", output, kp);

        // Detect sign change (zero crossing)
        if ((setpoint - output) * (setpoint - last_output) < 0) {
            if (!first_peak_detected) {
                first_peak_time = last_peak_time;
                first_peak_detected = 1;
            } else {
                tc = (last_peak_time - first_peak_time) / (oscillation_count * 1e6f);
            }
            oscillation_count++;
            last_peak_time = tick();
        }

        last_output = output;
        kp += 0.001;  // Increase kp to find instability
        delay(10);  // Sleep 10ms
    }

    kc = kp;

    // Calculate PID parameters - based on Ziegler-Nichols. Coefficients have been slightly adjusted.
    kp = 0.06 * kc;
    ki = 2 * (kp) / tc;
    kd = (kp) * (tc / 8);

    Serial.printf("kp%f, ki:%f, kd:%f\n", kp, ki, kd);
}

double pid(double setpoint, double measured, double kp, double ki, double kd, double dt) {
    static double integral = 0, prev_error = 0;
    double error = setpoint - measured;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}