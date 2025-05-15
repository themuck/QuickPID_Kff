/*  QuickPID.h - QuickPID Library for Arduino
    Copyright (c) 2019, 2023 Dlloydev. All right reserved.
    // ... (Lizenz etc. bleibt gleich) ...
*/

#ifndef QuickPID_h
#define QuickPID_h
#define LIBRARY_VERSION "3.0.2" // Inkrementiere Version für diese Änderung

#include <Arduino.h>

class QuickPID {

  public:

    // ################### Enums ###################
    enum class pMode { // Proportional mode
      pOnError = 0, pOnMeas = 1
    };

    enum class dMode { // Derivative mode
      dOnError = 0, dOnMeas = 1
    };

    enum class iAwMode { // Integral anti-windup mode
      iAwCondition = 0, iAwClamp = 1, iAwOff = 2
    };

    enum class cMode { // Controller mode
      manual = 0, automatique = 1, timer = 2
    };

    enum class cDir { // Controller direction
      dDirect = 0, dReverse = 1
    };

    // NEU: Enum für Feedforward-Richtung
    enum class ffDir { // Feedforward direction
      ffDirect = 0,    // Störgröße wird direkt zum Output addiert (mit Kff)
      ffReverse = 1    // Störgröße wird vom Output subtrahiert (mit Kff) - effektiv
    };

    // ############## Constructor ################
    // ... (Konstruktoren bleiben gleich) ...
    QuickPID(float *Input, float *Output, float *Setpoint);
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, pMode pMode, cDir cDir);
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, dMode dMode, cDir cDir);
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, pMode pMode, dMode dMode, iAwMode iAwMode, cDir cDir);
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float PMin, float PMax, float IMin, float IMax, float DMin, float DMax, pMode pMode, dMode dMode, iAwMode iAwMode, cDir cDir);
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, uint32_t SampleTimeUs, pMode pMode, dMode dMode, iAwMode iAwMode, cDir cDir);
    ~QuickPID();

    float* myInput;
    float* myOutput;
    float* mySetpoint;

    void SetTunings(float Kp, float Ki, float Kd);
    void SetTunings(float Kp, float Ki, float Kd, pMode pMode);
    void SetTunings(float Kp, float Ki, float Kd, dMode dMode);
    void SetTunings(float Kp, float Ki, float Kd, pMode pMode, dMode dMode, iAwMode iAwMode);
    // ... (Andere Getter/Setter bleiben gleich) ...
    void SetOutputLimits(float min, float max);
    float GetOutputMin();
    float GetOutputMax();
    void UsePOnError(bool POnError);
    void Reset();
    void Offset(float bias);
    float GetBias();
    bool Compute();

    // Erweiterung für Störgrößenaufschaltung
    void SetFeedforwardTunings(float Kff);
    void SetFeedforwardInput(float *ffInput);
    float GetKff();
    void SetFeedforwardDirection(ffDir direction); // NEU
    ffDir GetFeedforwardDirection();              // NEU

  private:

    void Initialize();
    void Controller(cDir cDir);
    bool CheckPointers();
    // ... (Andere private Member bleiben gleich) ...
    float _timerPid;
    float _bias;
    bool _mode, _options;
    pMode _pMode;
    dMode _dMode;
    iAwMode _iAwMode;
    cDir _controllerDirection;

    // Erweiterung für Störgrößenaufschaltung
    float *_feedforwardInput = nullptr;
    float _Kff = 0;
    ffDir _feedforwardDirection = ffDir::ffDirect; // NEU: Initialisierung hier

    // ... (Private Enums pOnError_e etc. bleiben gleich) ...
    typedef enum {
      pOnError_e = 0, pOnMeas_e = 1
    } PMODE;

    typedef enum {
      dOnError_e = 0, dOnMeas_e = 1
    } DMODE;

    typedef enum {
      iAwCondition_e = 0, iAwClamp_e = 1, iAwOff_e = 2
    } IAWMODE;

    typedef enum {
      manual_e = 0, automatique_e = 1, timer_e = 2
    } CMODE;

    typedef enum {
      dDirect_e = 0, dReverse_e = 1
    } CDIR;
};
#endif
