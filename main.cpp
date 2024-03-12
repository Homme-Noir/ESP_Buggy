#include "Motor.h"
#include "QEI.h"
#include "PID.h"
#include "mbed.h"

// Define motors and quadrature encoders
Motor leftMotor(PB_8, PB_9, PB_10);  // pwm, inB, inA
Motor rightMotor(PC_9, PC_8, PC_7); // pwm, inA, inB
QEI leftQei(PA_0, PA_1, NC, 624);  // chanA, chanB, index, ppr
QEI rightQei(PA_2, PA_3, NC, 624); // chanB, chanA, index, ppr

// Define the TCRT5000 sensor pins
AnalogIn sensorCenter(A0);
AnalogIn sensorLeft1(A1);
AnalogIn sensorLeft2(A2);
AnalogIn sensorRight1(A3);
AnalogIn sensorRight2(A4);
AnalogIn sensorBack(A5);

// Define the PID controller for line following
PID linePid(0.5, 0.1, 0.0, 0.01); // Example PID parameters

// Define base speed
const float baseSpeed = 0.5; // Adjust this value based on your requirements

float calculatePositionError(float center, float left1, float left2, float right1, float right2, float back) {
    // Constants for sensor positions relative to the center
    const float sensorSpacing = 12.0; // in mm

    // Calculate weighted position error
    float positionError = (center * 0 + left1 * (-sensorSpacing) + left2 * (-2 * sensorSpacing) +
                           right1 * sensorSpacing + right2 * (2 * sensorSpacing)) /
                          (center + left1 + left2 + right1 + right2);

    return positionError;
}

int main() {
    while (true) {
        // Read sensor values
        float centerValue = sensorCenter.read();
        float left1Value = sensorLeft1.read();
        float left2Value = sensorLeft2.read();
        float right1Value = sensorRight1.read();
        float right2Value = sensorRight2.read();
        float backValue = sensorBack.read();

        // Calculate the weighted average to determine the position error
        float positionError = calculatePositionError(centerValue, left1Value, left2Value, right1Value, right2Value, backValue);

        // Update the PID controller with the position error
        linePid.setProcessValue(positionError);

        // Compute new speeds for the motors based on PID output
        float correction = linePid.compute();
        leftMotor.speed(baseSpeed - correction);
        rightMotor.speed(baseSpeed + correction);

    }
}
