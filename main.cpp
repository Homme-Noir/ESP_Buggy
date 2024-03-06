#include "PID.h" // Include the PID controller class definition
#include "C12832.h" // Include the LCD display class definition

#define RATE 0.1 // Sampling rate for the PID controller

// Create an instance of the PID controller with initial parameters
PID controller(1.0, 0.0, 0.0, RATE);

// Analog input for the process variable
AnalogIn pv(PC_4);

// PWM output for the controller output
PwmOut co(PC_8);

// LCD display object initialization
C12832 lcd(D11, D13, D12, D7, D10);

int main() {
    // Configure input and output limits for the PID controller
    controller.setInputLimits(0.0, 3.3); // Analog input range (0.0V to 3.3V)
    controller.setOutputLimits(0.0, 1.0); // PWM output range (0.0 to 1.0)

    // Set bias, mode, and setpoint for the PID controller
    controller.setBias(0.3); // Set controller bias
    controller.setMode(AUTO_MODE); // Set controller mode to automatic
    controller.setSetPoint(1.7); // Set desired setpoint for the controller

    while(1) {
        // Read the process variable from the analog input
        controller.setProcessValue(pv.read());

        // Compute the new controller output
        float output = controller.compute();

        // Set the new output to the PwmOut object
        co = output;

        // Display the controller output on the LCD screen
        lcd.cls(); // Clear LCD screen
        lcd.locate(15, 10); // Set cursor position
        lcd.printf("%.2f", output); // Print the controller output with two decimal places

        // Wait for the specified sampling rate before the next iteration
        wait(RATE);
    }
}
