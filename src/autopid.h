#include <Arduino.h>

// Function to determine critical gain (Kc) and oscillation period (Tc)
void findCriticalValues(double *Kc, double *Tc, double(*process)(double)) {
    double setpoint = 100, output = 0, prev_output = 0;
    double Kp = 1.0;  // Initial proportional gain
    int oscillation_count = 0;
    double last_peak_time = time_us_32(), first_peak_time = time_us_32();
    int first_peak_detected = 0;

    Serial.printf("Finding Kc and Tc...\n");

    while (oscillation_count < 5) {  // Wait for 5 oscillations
        double control = Kp * (setpoint - output);
        output = process(control);

        Serial.printf("RPM: %f, Kp: %f\n", output, Kp);

        if ((setpoint - output) * (setpoint - prev_output) < 0) {  // Detect sign change (zero crossing)
            if (!first_peak_detected) {
                first_peak_time = last_peak_time;
                first_peak_detected = 1;
            } else {
                *Tc = (last_peak_time - first_peak_time) / (oscillation_count * 1e6f);
            }
            oscillation_count++;
            last_peak_time = time_us_32();
        }

        prev_output = output;
        Kp += 0.001;  // Increase Kp to find instability
        delay(10);  // Sleep 100ms
    }

    *Kc = Kp;
    Serial.printf("Critical Gain (Kc): %f, Oscillation Period (Tc): %f\n", *Kc, *Tc);
}

// Function to calculate PID parameters based on Ziegler-Nichols
void calculatePID(double Kc, double Tc, double *Kp, double *Ki, double *Kd) {
    *Kp = 0.06 * Kc;
    *Ki = 2 * (*Kp) / Tc;
    *Kd = (*Kp) * (Tc / 8);

    Serial.printf("Kp: %f, Ki: %f, Kd: %f\n", *Kp, *Ki, *Kd);
}