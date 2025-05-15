#include "QuickPID_Kff.h" // Assuming QuickPID_Kff.h and .cpp are in the same sketch folder or a library folder

// Define Variables we'll be connecting to
float Setpoint, Input, Output, FeedForwardInput;

// Define the aggressive and conservative Tuning Parameters
// These will need to be tuned for your specific system
float Kp = 2, Ki = 5, Kd = 1, Kff = 1; // Example tuning parameters

// Specify the links and initial tuning parameters
// Using the comprehensive constructor:
// QuickPID(float *Input, float *Output, float *Setpoint, float *FeedForward,
//          float Kp, float Ki, float Kd, float Kff,
//          pMode pMode, dMode dMode, iAwMode iAwMode, Action Action, ffAction ffAction);
QuickPID myPID(&Input, &Output, &Setpoint, &FeedForwardInput,
             Kp, Ki, Kd, Kff,
             QuickPID::pMode::pOnError,      // Proportional on Error
             QuickPID::dMode::dOnMeas,       // Derivative on Measurement
             QuickPID::iAwMode::iAwCondition, // Anti-windup conditional
             QuickPID::Action::direct,       // Controller Action: Direct (output increases with error)
             QuickPID::ffAction::direct);    // Feed-forward Action: Direct

unsigned long lastTime;
unsigned long windowStartTime;
int cycleCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect. Needed for native USB port only

  Input = 20; // Simulate initial room temperature
  Setpoint = 50; // Desired temperature
  FeedForwardInput = 0; // Initial disturbance

  // Set output limits. For example, a heater might output 0-255 for PWM
  myPID.SetOutputLimits(0, 255);

  // Turn the PID on
  myPID.SetMode(QuickPID::Control::automatic);

  Serial.println("QuickPID Example Started");
  Serial.println("------------------------------------");
  Serial.println("Demonstrating Feed-Forward with ffAction::direct initially.");

  lastTime = millis();
  windowStartTime = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);

  if (timeChange >= 1000) { // Update every 1 second for simulation
    cycleCount++;

    // Simulate a process:
    // Input (temperature) slowly changes based on Output (heater) and disturbances
    // This is a very simple simulation. A real system would be more complex.
    Input += (Output / 50.0) * (timeChange / 1000.0); // Heater effect
    Input -= 0.1 * (timeChange / 1000.0); // Natural cooling
    Input -= FeedForwardInput * 0.2 * (timeChange / 1000.0); // Disturbance effect

    // Simulate a disturbance (e.g., window opens for a period)
    if (cycleCount > 10 && cycleCount <= 20) {
      FeedForwardInput = 5.0; // Window open, causing heat loss
      if (cycleCount == 11) Serial.println("\n!!! Disturbance: Window Opened (FeedForwardInput = 5.0) !!!\n");
    } else {
      FeedForwardInput = 0.0; // Window closed
      if (cycleCount == 21) Serial.println("\n!!! Disturbance: Window Closed (FeedForwardInput = 0.0) !!!\n");
    }

    // Change Feed-Forward direction after some time to demonstrate
    if (cycleCount == 30) {
      myPID.SetFeedForwardDirection(QuickPID::ffAction::reverse);
      Serial.println("------------------------------------");
      Serial.println("Changed Feed-Forward Direction to ffAction::reverse");
      Serial.println("Now, a positive FeedForwardInput will DECREASE the corrective action.");
      Serial.println("This might be used if FF input was 'cooling power available' instead of 'heat loss'");
      Serial.println("------------------------------------");
    }
     if (cycleCount == 35 && cycleCount <= 45) {
      FeedForwardInput = 3.0; // Simulate a different kind of FF input
      if (cycleCount == 35) Serial.println("\n!!! Disturbance with REVERSE FF (FeedForwardInput = 3.0) !!!\n");
    } else if (cycleCount > 45 && FeedForwardInput != 0.0) {
        FeedForwardInput = 0.0;
        if (cycleCount == 46) Serial.println("\n!!! Disturbance with REVERSE FF ends !!!\n");
    }


    bool computed = myPID.Compute(); // Calculate PID output

    if (computed) {
      Serial.print("Cycle: "); Serial.print(cycleCount);
      Serial.print(" | Setpoint: "); Serial.print(Setpoint);
      Serial.print(" | Input: "); Serial.print(Input);
      Serial.print(" | Output: "); Serial.print(Output);
      Serial.print(" | FFInput: "); Serial.print(FeedForwardInput);
      Serial.print(" | FFTerm: "); Serial.print(myPID.GetFFterm());
      Serial.print(" | FFAction: "); Serial.println((myPID.GetFeedForwardDirection() == static_cast<uint8_t>(QuickPID::ffAction::direct)) ? "direct" : "reverse");
    }
    lastTime = now;
  }
}
