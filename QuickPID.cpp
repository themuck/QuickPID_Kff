/* QuickPID.cpp - QuickPID Library for Arduino
    // ... (Lizenz etc. bleibt gleich) ...
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID.h"

// ... (Alle Konstruktoren bleiben unverändert in ihrer Signatur) ...
QuickPID::QuickPID(float *Input, float *Output, float *Setpoint) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  Initialize();
}
// ... (Andere Konstruktoren) ...
QuickPID::QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, uint32_t SampleTimeUs, pMode PMode, dMode DMode, iAwMode IAwMode, cDir CDir) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  _pMode = PMode;
  _dMode = DMode;
  _iAwMode = IAwMode;
  _controllerDirection = CDir;
  _sampleTimeUs = SampleTimeUs;
  Initialize();
  SetTunings(Kp, Ki, Kd);
}

QuickPID::~QuickPID() {}

// ... (SetTunings und die meisten Getter/Setter bleiben gleich) ...
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  if ((Kp < 0) || (Ki < 0) || (Kd < 0)) return;
  _Kp = Kp; _Ki = Ki; _Kd = Kd;
  Controller(_controllerDirection);
}
// ...

void QuickPID::Initialize() {
  _lastTime = micros() - _sampleTimeUs;
  PIDsum = 0;
  pTerm = 0;
  iTerm = 0;
  dTerm = 0;
  lastInput = 0;
  prevError = 0;
  _bias = 0;
  // Feedforward spezifisch
  _feedforwardInput = nullptr;
  _Kff = 0;
  _feedforwardDirection = ffDir::ffDirect; // Sicherstellen der Initialisierung
}

// ... (SetMode, die meisten Getter/Setter bleiben gleich) ...

bool QuickPID::Compute() {
  if (!CheckPointers()) return false;
  
  uint32_t now = micros();
  uint32_t timeChange = (now - _lastTime);

  if (_mode == cMode::timer) {
    if (timeChange >= _sampleTimeUs) {
      goto procceed;
    } else return false;
  } else goto procceed;

procceed:
  _timerPid = timeChange / 1000000.0f;

  if (!_mode) {
    // ... (Manual mode logic bleibt gleich) ...
    return true;
  }

  float input = *myInput;
  float setpoint = *mySetpoint;
  error = setpoint - input;

  /*P Term*/
  // ... (P Term logic bleibt gleich) ...
  if (_pMode == pMode::pOnError) {
    pTerm = _Kp * error;
  } else {
    pTerm = _Kp * (input);
    pTerm = -pTerm;
  }
  if (_pMin || _pMax) pTerm = constrain(pTerm, (_pMin * _pFac), (_pMax * _pFac));


  /*I Term*/
  // ... (I Term logic bleibt gleich) ...
  if (_Ki != 0) {
    if (_iAwMode == iAwMode::iAwCondition) {
      if (_controllerDirection == cDir::dDirect && (input < setpoint)) iTerm = _Ki * error * _timerPid;
      else if (_controllerDirection == cDir::dReverse && (input > setpoint)) iTerm = _Ki * error * _timerPid;
      else iTerm = 0;
    } else iTerm = _Ki * error * _timerPid;
    if (_iMin || _iMax) iTerm = constrain(iTerm, _iMin, _iMax);
  } else iTerm = 0;


  /*D Term*/
  // ... (D Term logic bleibt gleich) ...
  if (_Kd != 0) {
    if (_dMode == dMode::dOnError) {
      dTerm = _Kd * (error - prevError) / _timerPid;
    } else {
      dTerm = _Kd * (input - lastInput) / _timerPid;
      dTerm = -dTerm;
    }
    if (_dMin || _dMax) dTerm = constrain(dTerm, (_dMin * _dFac), (_dMax * _dFac));
  } else dTerm = 0;


  lastInput = input;
  prevError = error;

  // Anti-windup and Sum
  // ... (PIDsum logic bleibt gleich) ...
  if (_iAwMode == iAwMode::iAwClamp && (_Ki != 0)) {
    float iTermTemp = PIDsum + iTerm;
    if (iTermTemp > _outputMax) PIDsum = max(_outputMax, PIDsum);
    else if (iTermTemp < _outputMin) PIDsum = min(_outputMin, PIDsum);
    else PIDsum = iTermTemp;
  } else PIDsum += iTerm;


  // Berechnung des Feedforward-Anteils MIT RICHTUNG
  float ffTerm = 0;
  if (_feedforwardInput != nullptr && _Kff != 0) {
    ffTerm = _Kff * (*_feedforwardInput);
    if (_feedforwardDirection == ffDir::ffReverse) {
      ffTerm = -ffTerm;
    }
  }
  
  /*compute output*/
  float output = _pTerm + PIDsum + _dTerm + _bias + ffTerm;

  // Output Limiting
  // ... (Output limiting logic bleibt gleich) ...
  if (_outputMin || _outputMax) {
#if defined(sympathetic)
    output = constrain(output, _outputMin, _outputMax);
#else
    if (output < _outputMin) output = _outputMin;
    else if (output > _outputMax) output = _outputMax;
    else if (output < _outputMax && output > _outputMin);
    else if (setpoint > input) output = _outputMax;
    else if (setpoint < input) output = _outputMin;
#endif
  }
  *myOutput = output;
  _lastTime = now;
  return true;
}

// ... (SetSampleTimeUs, GetSampleTimeUs, SetOutputLimits, etc. bleiben gleich) ...
void QuickPID::SetSampleTimeUs(uint32_t SampleTimeUs) {
  if (SampleTimeUs > 0) _sampleTimeUs = SampleTimeUs;
}
//...

void QuickPID::Reset() {
  PIDsum = 0;
  pTerm = 0;
  iTerm = 0;
  dTerm = 0;
  lastInput = 0;
  prevError = 0;
  _bias = 0;
  // Kff, _feedforwardInput und _feedforwardDirection werden bei Reset NICHT zurückgesetzt,
  // da sie Konfigurationsparameter sind.
}

// ... (Offset, GetBias, CheckPointers, Controller bleiben gleich) ...

// METHODEN für Störgrößenaufschaltung
void QuickPID::SetFeedforwardTunings(float Kff) {
  _Kff = Kff;
}

void QuickPID::SetFeedforwardInput(float *ffInput) {
  _feedforwardInput = ffInput;
}

float QuickPID::GetKff() {
  return _Kff;
}

void QuickPID::SetFeedforwardDirection(ffDir direction) { // NEU
  _feedforwardDirection = direction;
}

QuickPID::ffDir QuickPID::GetFeedforwardDirection() { // NEU
  return _feedforwardDirection;
}
