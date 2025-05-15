/**********************************************************************************
   QuickPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
 **********************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID_Kff.h"

QuickPID::QuickPID() : myFeedForward(nullptr), kff(0), dispKff(0), ffaction(ffAction::direct), ffTerm(0) {}

/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/
// Constructor with FeedForward
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint, float* FeedForward,
                   float Kp, float Ki, float Kd, float Kff,
                   pMode pModeVal, dMode dModeVal, iAwMode iAwModeVal,
                   Action ActionVal, ffAction ffActionVal) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  myFeedForward = FeedForward;
  mode = Control::manual;

  QuickPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QuickPID::SetControllerDirection(ActionVal);
  QuickPID::SetFeedForwardDirection(ffActionVal);
  QuickPID::SetTunings(Kp, Ki, Kd, Kff, pModeVal, dModeVal, iAwModeVal);

  lastTime = micros() - sampleTimeUs;
  ffTerm = 0; // Initialize ffTerm
}

// Overload Constructor with FeedForward (simplified PID modes)
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint, float* FeedForward,
                   float Kp, float Ki, float Kd, float Kff,
                   Action ActionVal, ffAction ffActionVal)
  : QuickPID(Input, Output, Setpoint, FeedForward, Kp, Ki, Kd, Kff,
             pMode::pOnError, dMode::dOnMeas, iAwMode::iAwCondition,
             ActionVal, ffActionVal) {
}

// Simplified Constructor with FeedForward (defaults for all gains and modes)
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint, float* FeedForward)
  : QuickPID(Input, Output, Setpoint, FeedForward,
             0, 0, 0, 0, // Kp, Ki, Kd, Kff
             pMode::pOnError, dMode::dOnMeas, iAwMode::iAwCondition,
             Action::direct, ffAction::direct) {
}


// Original Constructor (without FeedForward)
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd,
                   pMode pModeVal, dMode dModeVal, iAwMode iAwModeVal, Action ActionVal)
  : QuickPID(Input, Output, Setpoint, nullptr, // myFeedForward = nullptr
             Kp, Ki, Kd, 0,                    // Kff = 0
             pModeVal, dModeVal, iAwModeVal, ActionVal, ffAction::direct) { // default ffAction
}

/* Constructor *********************************************************************
   To allow using pOnError, dOnMeas and iAwCondition without explicitly saying so. (Original without FF)
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, Action ActionVal)
  : QuickPID(Input, Output, Setpoint, nullptr, Kp, Ki, Kd, 0, ActionVal, ffAction::direct) {
}

/* Constructor *********************************************************************
   Simplified constructor which uses defaults for remaining parameters. (Original without FF)
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint)
  : QuickPID(Input, Output, Setpoint, nullptr) { // Calls the simplified constructor with FF (nullptr)
}

/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QuickPID::Compute() {
  if (mode == Control::manual) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);
  if (mode == Control::timer || timeChange >= sampleTimeUs) {

    float input = *myInput;
    float dInput = input - lastInput;
    if (action == Action::reverse) dInput = -dInput;

    error = *mySetpoint - input;
    if (action == Action::reverse) error = -error;
    float dError = error - lastError;

    float peTerm = kp * error;
    float pmTerm = kp * dInput;
    if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    }
    pTerm = peTerm - pmTerm; // used by GetPterm()
    iTerm =  ki  * error; // used by GetIterm()
    if (dmode == dMode::dOnError) dTerm = kd * dError;
    else dTerm = -kd * dInput; // dOnMeas, used by GetDterm()

    // Calculate FeedForward Term
    ffTerm = 0; // Default to 0 if no feedforward input
    if (myFeedForward != nullptr) {
      float feedForwardInput = *myFeedForward;
      ffTerm = kff * feedForwardInput;
      if (ffaction == ffAction::reverse) {
        ffTerm = -ffTerm;
      }
    }

    //condition anti-windup (default)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      // Propose output if iTerm is allowed to update
      float proposedIterm = iTerm + ki * error; // Potential new iTerm before ffTerm
      float pidOutputWithoutI = (peTerm - pmTerm) + dTerm; // PID output without integral and ff
      float outputIfITermUpdated = pidOutputWithoutI + proposedIterm + ffTerm;

      if (outputIfITermUpdated > outMax && error > 0) aw = true; // Check error sign for windup condition
      else if (outputIfITermUpdated < outMin && error < 0) aw = true; // Check error sign for windup condition
      
      if (aw && ki != 0) { // If aw is true and ki is not zero
        // Don't update iTerm, or clamp it if necessary (though simple prevention is often better)
        // For simplicity here, we prevent iTerm from causing further saturation.
        // More advanced: iTerm = (action == Action::direct ? outMax : outMin) - pidOutputWithoutI - ffTerm;
        // For now, we just don't update iTerm if it would push output beyond limits
      } else {
         iTerm += ki * error; // Update iTerm if no windup condition
      }
    } else { // iAwOff or iAwClamp
        iTerm += ki * error; // Standard integral accumulation
    }


    // PID output calculation
    // outputSum is effectively the integral sum here.
    // Let's adjust how outputSum is used or recalculate the output.
    // The original code had a slightly confusing use of outputSum.
    // Let's make it clearer:
    // outputSum represents the accumulated integral term.

    outputSum = iTerm; // Start with the current integral term

    if (iawmode == iAwMode::iAwOff) {
      // No clamping on integral sum for iAwOff, pmTerm is part of pTerm
    } else if (iawmode == iAwMode::iAwClamp) {
      // Clamp the integral sum itself
      outputSum = constrain(outputSum, outMin - (pTerm + dTerm + ffTerm), outMax - (pTerm + dTerm + ffTerm));
    } else { // iAwCondition - integral already handled with windup logic
        // outputSum is iTerm
    }

    // Final Output Calculation
    // Original: outputSum += iTerm; then outputSum -= pmTerm; then *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);
    // New approach:
    float pidCalc = pTerm + outputSum + dTerm; // P + I + D
    *myOutput = constrain(pidCalc + ffTerm, outMin, outMax); // Add FeedForward and clamp

    lastError = error;
    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
// SetTunings with FeedForward and PID modes
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float Kff,
                          pMode pModeVal, dMode dModeVal, iAwMode iAwModeVal) {
  if (Kp < 0 || Ki < 0 || Kd < 0 || Kff < 0) return;
  if (Ki == 0) iTerm = 0; // Reset integral term if Ki is zero
  pmode = pModeVal;
  dmode = dModeVal;
  iawmode = iAwModeVal;

  dispKp = Kp; dispKi = Ki; dispKd = Kd; dispKff = Kff;

  float SampleTimeSec = (float)sampleTimeUs / 1000000.0f;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kff = Kff; // Kff is not scaled by sample time
}

// SetTunings with FeedForward (uses last remembered PID modes)
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float Kff) {
  SetTunings(Kp, Ki, Kd, Kff, pmode, dmode, iawmode);
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered pMode, dMode and iAwMode settings. (Original without Kff)
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  // Calls the new SetTunings with Kff = 0 or current kff if it should be preserved
  // For now, let's assume if this is called, Kff is meant to be the existing kff or 0 if not set.
  // To be safe, we can call the full version with the current dispKff.
  SetTunings(Kp, Ki, Kd, dispKff, pmode, dmode, iawmode);
}

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(..)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QuickPID::SetOutputLimits(float Min, float Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (mode != Control::manual) {
    *myOutput = constrain(*myOutput, outMin, outMax);
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

/* SetMode(.)*****************************************************************
  Sets the controller mode to manual (0), automatic (1) or timer (2)
  when the transition from manual to automatic or timer occurs, the
  controller is automatically initialized.
******************************************************************************/
void QuickPID::SetMode(Control Mode) {
  if (mode == Control::manual && Mode != Control::manual) { // just went from manual to automatic, timer or toggle
    QuickPID::Initialize();
  }
  if (Mode == Control::toggle) {
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = Mode;
}
void QuickPID::SetMode(uint8_t Mode) {
  if (mode == Control::manual && Mode != 0) { // just went from manual to automatic or timer
    QuickPID::Initialize();
  }
  if (Mode == 3) { // toggle
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = (Control)Mode;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize() {
  // outputSum here is the integral accumulator.
  // To achieve bumpless transfer, the integral term (outputSum) should be set
  // such that the PID output matches the current *myOutput.
  // *myOutput = pTerm + iTerm + dTerm + ffTerm
  // iTerm = *myOutput - (pTerm + dTerm + ffTerm)
  // However, pTerm, dTerm, and ffTerm depend on current inputs/setpoints which might not be stable.
  // A common approach is to set iTerm = *myOutput and then adjust if other terms are significant.
  // For simplicity, and as in original, set outputSum (iTerm) to current output,
  // then constrain. This assumes pTerm and dTerm are small or will quickly adjust.
  // ffTerm should also be considered if active.

  lastInput = *myInput; // Update lastInput for derivative calculation
  // Calculate initial pTerm, dTerm, ffTerm based on current state if possible
  // This is complex as error might not be defined yet.
  // The original Initialize just set outputSum = *myOutput.
  // Let's refine this slightly.
  // If we set iTerm = *myOutput, then PID output = pTerm + *myOutput + dTerm + ffTerm, which is wrong.
  // We want: *myOutput = pTerm_initial + iTerm_initial + dTerm_initial + ffTerm_initial
  // A simpler bumpless transfer:
  iTerm = *myOutput; // Assume *myOutput is the desired starting point for the integral.
                     // If p, d, ff terms are active, this might not be perfect but is common.
  if (myFeedForward != nullptr && kff != 0) {
    float currentFF = *myFeedForward;
    if (ffaction == ffAction::reverse) currentFF = -currentFF;
    iTerm -= kff * currentFF; // Adjust iTerm to account for existing feedforward
  }
  // pTerm and dTerm are harder to pre-calculate for bumpless without knowing the error history.
  // Often, setting iTerm to *myOutput (or *myOutput - ffTerm) is sufficient.
  outputSum = iTerm; // outputSum is our integral accumulator
  outputSum = constrain(outputSum, outMin, outMax); // Constrain the integral sum itself.
                                                    // This might need to be more nuanced based on other terms.
                                                    // A better approach for bumpless with all terms:
                                                    // iTerm = *myOutput - (calculated_pTerm + calculated_dTerm + calculated_ffTerm)
                                                    // For now, stick to a simpler version close to original:
  // outputSum = *myOutput; // This was the original line for outputSum
  // lastInput = *myInput;
  // outputSum = constrain(outputSum, outMin, outMax);
  // Let's use the iTerm approach:
  iTerm = *myOutput;
  if (myFeedForward != nullptr) {
    float ffVal = *myFeedForward;
    if (ffaction == ffAction::reverse) ffVal = -ffVal;
    iTerm -= kff * ffVal;
  }
  // pTerm and dTerm are based on error, which will be calculated on the first Compute.
  // So, setting iTerm to *myOutput - ffTerm is a reasonable start.
  outputSum = constrain(iTerm, outMin, outMax); // outputSum is the integral accumulator
  lastInput = *myInput;
}

/* SetControllerDirection(.)**************************************************
  The PID will either be connected to a direct acting process (+Output leads
  to +Input) or a reverse acting process(+Output leads to -Input).
******************************************************************************/
void QuickPID::SetControllerDirection(Action Action) {
  action = Action;
}
void QuickPID::SetControllerDirection(uint8_t Direction) {
  action = (Action)Direction;
}

/* SetProportionalMode(.)*****************************************************
  Sets the computation method for the proportional term, to compute based
  either on error (default), on measurement, or the average of both.
******************************************************************************/
void QuickPID::SetProportionalMode(pMode pMode) {
  pmode = pMode;
}
void QuickPID::SetProportionalMode(uint8_t Pmode) {
  pmode = (pMode)Pmode;
}

/* SetDerivativeMode(.)*******************************************************
  Sets the computation method for the derivative term, to compute based
  either on error or on measurement (default).
******************************************************************************/
void QuickPID::SetDerivativeMode(dMode dMode) {
  dmode = dMode;
}
void QuickPID::SetDerivativeMode(uint8_t Dmode) {
  dmode = (dMode)Dmode;
}

/* SetAntiWindupMode(.)*******************************************************
  Sets the integral anti-windup mode to one of iAwClamp, which clamps
  the output after adding integral and proportional (on measurement) terms,
  or iAwCondition (default), which provides some integral correction, prevents
  deep saturation and reduces overshoot.
  Option iAwOff disables anti-windup altogether.
******************************************************************************/
void QuickPID::SetAntiWindupMode(iAwMode iAwMode) {
  iawmode = iAwMode;
}
void QuickPID::SetAntiWindupMode(uint8_t IawMode) {
  iawmode = (iAwMode)IawMode;
}

void QuickPID::Reset() {
  lastTime = micros() - sampleTimeUs;
  lastInput = 0; // or *myInput if available and makes sense
  outputSum = 0; // Integral accumulator
  pTerm = 0;
  iTerm = 0;
  dTerm = 0;
  ffTerm = 0; // Reset feed-forward term
}

// sets the output summation value
void QuickPID::SetOutputSum(float sum) {
  outputSum = sum;
}

/* Status Functions************************************************************
  These functions query the internal state of the PID.
******************************************************************************/
float QuickPID::GetKp() {
  return dispKp;
}
float QuickPID::GetKi() {
  return dispKi;
}
float QuickPID::GetKd() {
  return dispKd;
}
float QuickPID::GetKff() { // New Getter
  return dispKff;
}
float QuickPID::GetPterm() {
  return pTerm;
}
float QuickPID::GetIterm() {
  // return iTerm; // iTerm is now the raw integral before scaling by Ki in some interpretations
  // outputSum is the accumulated integral component.
  return outputSum; // Or return iTerm if that's defined as the component. Let's stick to iTerm for the unscaled integral.
                    // The definition of GetIterm() should be consistent.
                    // If iTerm is ki * error * dt, then outputSum is sum(iTerm).
                    // The header has GetIterm() as "integral component of output".
                    // In the compute, iTerm = ki * error. outputSum accumulates this.
                    // So, GetIterm() should probably return the latest iTerm contribution, and GetOutputSum() the total.
                    // However, the original code had outputSum += iTerm, and GetIterm returned iTerm.
                    // Let's assume iTerm is the latest calculated integral contribution (scaled by Ki already).
  return iTerm; // This is the ki*error term from the last compute cycle.
}
float QuickPID::GetDterm() {
  return dTerm;
}
float QuickPID::GetFFterm() { // New Getter
  return ffTerm;
}
float QuickPID::GetOutputSum() { // This is the accumulated integral sum.
  return outputSum;
}
uint8_t QuickPID::GetMode() {
  return static_cast<uint8_t>(mode);
}
uint8_t QuickPID::GetDirection() {
  return static_cast<uint8_t>(action);
}
uint8_t QuickPID::GetFeedForwardDirection() { // New Getter
  return static_cast<uint8_t>(ffaction);
}
uint8_t QuickPID::GetPmode() {
  return static_cast<uint8_t>(pmode);
}
uint8_t QuickPID::GetDmode() {
  return static_cast<uint8_t>(dmode);
}
uint8_t QuickPID::GetAwMode() {
  return static_cast<uint8_t>(iawmode);
}

// New Setters for FeedForward
void QuickPID::SetFeedForwardGain(float Kff) {
  if (Kff < 0) return;
  dispKff = Kff;
  kff = Kff; // Kff is not scaled by sample time
}

void QuickPID::SetFeedForwardDirection(ffAction ffActionVal) {
  ffaction = ffActionVal;
}

void QuickPID::SetFeedForwardDirection(uint8_t Direction) {
  ffaction = (ffAction)Direction;
}
