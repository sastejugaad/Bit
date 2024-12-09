#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <Arduino.h>

class HeadingController {
private:
    float kp, ki, kd; // PID gains
    float targetHeading; // Desired heading angle in degrees
    float integral;     // Integral term for PID
    float lastError;    // Last error value
    unsigned long lastTime; // Last time PID was computed

public:
    HeadingController() : kp(1), ki(0), kd(0), targetHeading(0), integral(0), lastError(0), lastTime(0) {}

    void setPID(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void setTargetHeading(float heading) {
        targetHeading = heading;
    }

    float getTargetHeading() const {
        return targetHeading;
    }

    float computePID(float currentHeading) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        if (lastTime == 0 || deltaTime <= 0) deltaTime = 0.01; // Prevent divide by zero

        // Calculate error and adjust for wrap-around (heading range: 0-360)
        float error = targetHeading - currentHeading;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        // Proportional term
        float pTerm = kp * error;

        // Integral term
        integral += error * deltaTime;
        float iTerm = ki * integral;

        // Derivative term
        float dTerm = kd * (error - lastError) / deltaTime;

        // Store current values for next iteration
        lastError = error;
        lastTime = currentTime;

        // PID output
        return pTerm + iTerm + dTerm;
    }

    void reset() {
        integral = 0;
        lastError = 0;
        lastTime = 0;
    }
};

#endif // HEADING_CONTROLLER_H
