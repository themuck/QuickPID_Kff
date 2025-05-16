/*
 * This library implements a PID controller with a FeedForward (Kff) component.
 * The FeedForward term is based on [Specify basis, e.g., a simplified model of the system, an expected disturbance, etc.].
 * Created by [Your Name], [Date]
 * Version 1.0
 */
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

    // Member 'iTerm' was set at L112 as (ki * error) for the current cycle. This is for GetIterm().
    // Member 'outputSum' is the integral accumulator from the previous cycle.
    float currentCycleIntegralContribution = iTerm; // Use the value set at L112.

    // Anti-windup logic: Update 'outputSum' (the integral accumulator)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      float pAndDTerms = pTerm + dTerm; // pTerm is (peTerm - pmTerm)
      // Calculate total potential output if integral accumulator (outputSum) is updated
      float potentialTotalOutput = pAndDTerms + (outputSum + currentCycleIntegralContribution) + ffTerm;

      if (potentialTotalOutput > outMax && error > 0) aw = true;
      else if (potentialTotalOutput < outMin && error < 0) aw = true;
      
      if (!aw || ki == 0) { // If no windup or Ki is zero, update accumulator
        outputSum += currentCycleIntegralContribution;
      }
      // else: outputSum (integral accumulator) is not updated this cycle due to windup
    } else { // iAwOff or iAwClamp (accumulation happens first for both)
        outputSum += currentCycleIntegralContribution;
    }

    // If iAwClamp mode, clamp the now-updated integral accumulator (outputSum).
    // This is a separate step after accumulation.
    if (iawmode == iAwMode::iAwClamp) {
      float nonIntegralTerms = pTerm + dTerm + ffTerm;
      outputSum = constrain(outputSum, outMin - nonIntegralTerms, outMax - nonIntegralTerms);
    }
    // For iAwOff, outputSum is unconstrained by this specific clamping.
    // For iAwCondition, outputSum was conditionally updated; no further clamping of outputSum itself here.
    // The final *myOutput will be constrained later.

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
  lastInput = *myInput; // Update lastInput for derivative calculation

  // To achieve bumpless transfer, the integral term (iTerm, which becomes outputSum)
  // should be set such that the PID output initially matches the current *myOutput.
  // We initialize iTerm = *myOutput and then subtract the feedforward component,
  // as pTerm and dTerm will be calculated on the first Compute() call.
  iTerm = *myOutput;

  if (myFeedForward != nullptr && kff != 0) { // Check kff to avoid unnecessary computation
    float ffVal = *myFeedForward;
    if (ffaction == ffAction::reverse) {
      ffVal = -ffVal;
    }
    iTerm -= kff * ffVal; // Adjust iTerm to account for existing feedforward
  }

  outputSum = constrain(iTerm, outMin, outMax); // outputSum is the integral accumulator
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
