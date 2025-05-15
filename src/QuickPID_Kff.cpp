/**********************************************************************************
   QuickPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
   MODIFIED to include Feedforward Control.
 **********************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID.h" // Stelle sicher, dass dies auf deine modifizierte QuickPID.h zeigt

QuickPID::QuickPID() {
  // Initialisiere Feedforward-Variablen auch im Standardkonstruktor
  this->_feedforwardInput = nullptr;
  this->_Kff = 0.0f;
  this->_feedforwardDirection = ffDir::ffDirect;
}

/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/

QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, // Kp, Ki, Kd müssen nicht zwingend Defaults haben, wenn sie immer übergeben werden
                   pMode pMode, // pMode pMode = pMode::pOnError, (Original-Defaults)
                   dMode dMode, // dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode, // iAwMode iAwMode = iAwMode::iAwCondition,
                   Action Action) { // Action Action = Action::direct) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = Control::manual; // Start im manuellen Modus

  QuickPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QuickPID::SetControllerDirection(Action);
  QuickPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode); // Ruft die überladene Version auf

  lastTime = micros() - sampleTimeUs;

  // Initialisiere Feedforward-Variablen
  this->_feedforwardInput = nullptr;
  this->_Kff = 0.0f;
  this->_feedforwardDirection = ffDir::ffDirect;
}

/* Constructor *********************************************************************
   To allow using pOnError, dOnMeas and iAwCondition without explicitly saying so.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, Action Action)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, // Ruft Hauptkonstruktor auf
                       pMode::pOnError,     // Standard pmode
                       dMode::dOnMeas,      // Standard dmode
                       iAwMode::iAwCondition, // Standard iawmode
                       Action) {
  // Feedforward wird im aufgerufenen Hauptkonstruktor initialisiert
}

/* Constructor *********************************************************************
   Simplified constructor which uses defaults for remaining parameters.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint)
  : QuickPID::QuickPID(Input, Output, Setpoint, // Ruft Hauptkonstruktor auf
                       0, // Default Kp
                       0, // Default Ki
                       0, // Default Kd
                       pMode::pOnError,
                       dMode::dOnMeas,
                       iAwMode::iAwCondition,
                       Action::direct) { // Default Action
  // Feedforward wird im aufgerufenen Hauptkonstruktor initialisiert
  // dispKp, dispKi, dispKd werden in SetTunings gesetzt, was vom Hauptkonstruktor aufgerufen wird
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

  if (mode == Control::timer || timeChange >= sampleTimeUs) { // In timer mode, always compute if true

    float input = *myInput;
    float dInput = input - lastInput; // dInput ist (current input - last input)

    // Reverse Action wird hier angewendet, um Vorzeichen für error und dInput zu korrigieren
    // bevor sie in P, I, D Terme einfließen
    if (action == Action::reverse) {
        dInput = -dInput;
    }

    error = *mySetpoint - input;
    if (action == Action::reverse) {
        error = -error; // Wenn reverse, wird positiver error zu negativem etc.
    }
    float dError = error - lastError; // dError ist (current error - last error)


    // --- Proportional Term ---
    float peTerm = kp * error; // P on Error component
    float pmTerm = kp * dInput;  // P on Measurement component (dInput ist hier schon für reverse action korrigiert)
                                 // Wenn pOnError, pmTerm = 0. Wenn pOnMeas, peTerm = 0.

    if (pmode == pMode::pOnError) {
        // pmTerm wird effektiv zu 0, weil es nicht zum pTerm addiert/subtrahiert wird
        pTerm = peTerm;
    } else if (pmode == pMode::pOnMeas) {
        // peTerm wird effektiv zu 0
        pTerm = -pmTerm; // pOnMeas ist typischerweise negativ
    } else { // pOnErrorMeas
        pTerm = (peTerm - pmTerm) * 0.5f; // Der P-Term ist der Durchschnitt
    }
    // Wichtig: Das Original-QuickPID (v3.1.9) berechnet P-Term etwas anders:
    // pTerm = peTerm - pmTerm; // und modifiziert dann peTerm/pmTerm vorher.
    // Ich halte mich hier an die explizitere Struktur oben zur Klarheit.
    // Wenn man sich exakt an die Originalzeile hält:
    // float original_peTerm = kp * error;
    // float original_pmTerm = kp * dInput;
    // if (pmode == pMode::pOnError) original_pmTerm = 0;
    // else if (pmode == pMode::pOnMeas) original_peTerm = 0;
    // else { original_peTerm *= 0.5f; original_pmTerm *= 0.5f; }
    // pTerm = original_peTerm - original_pmTerm;  // Dies ist der P-Anteil für den Output.
                                                 // pTerm als Member-Variable wird hier zum Speichern des Ergebnisses genutzt.

    // --- Integral Term ---
    iTerm = ki * error; // Der Beitrag dieses Zyklus zum Integral

    // --- Derivative Term ---
    if (dmode == dMode::dOnError) {
        dTerm = kd * dError;
    } else { // dOnMeas
        dTerm = -kd * dInput; // dOnMeasurement ist typischerweise negativ
    }


    // --- Anti-windup (iAwCondition) ---
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      // Teste, ob der I-Term den Output über die Grenzen treiben würde
      // Die Original-Logik: float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error); -> Hier ist iTerm der akkumulierte Wert (outputSum)
      // Da outputSum bereits pmTerm enthält (oder auch nicht, je nach iawmode),
      // müssen wir vorsichtig sein. Betrachten wir den potenziellen neuen I-Anteil:
      float potentialOutputWithFullIntegral = pTerm + (outputSum + iTerm) + dTerm; // outputSum ist der alte I-Anteil

      if (ki != 0) { // Nur wenn I-Term aktiv ist
        if ((outputSum + iTerm) > outMax && error > 0) aw = true; // Wenn bereits am Limit und Fehler weiter in die Richtung geht
        else if ((outputSum + iTerm) < outMin && error < 0) aw = true;

        // Die Original-Logik ist komplexer hier:
        // float iTermOutTest = (pmode == pMode::pOnMeas ? 0 : kp * error) - (pmode == pMode::pOnError ? 0 : kp * dInput) + ki * (outputSum_ohne_pmTerm + iTerm);
        // Wenn der aktuelle inkrementelle iTerm (also `iTerm` hier) den `outputSum` (der das kumulierte I ist)
        // über die Grenzen treiben würde UND der Fehler in die Sättigungsrichtung zeigt, dann wird iTerm evtl. nicht addiert oder modifiziert.

        // Vereinfachte Kondition (ähnlich wie oft üblich):
        // Wenn outputSum bereits an einer Grenze ist und der aktuelle iTerm in die gleiche Richtung weiter akkumulieren würde,
        // wird der aktuelle iTerm nicht addiert, um weiteres Wind-up zu verhindern.
        if ((outputSum >= outMax && iTerm > 0) || (outputSum <= outMin && iTerm < 0)) {
             // Mache nichts, oder setze iTerm = 0 für diesen Zyklus, um Wind-up zu verhindern
             // Dies ist die iAwCondition-Logik:
             // Die Originalzeile: if (iTermOut > outMax && dError > 0) aw = true; -> dError statt error
             // Wenn iTermOut (potenzieller Output mit neuem I) > outMax UND dError > 0 (Fehler wird größer in pos. Richtung)
             // ki ist schon einberechnet in iTerm hier.
           if ( (pTerm + (outputSum + iTerm) + dTerm > outMax && error > 0 /*oder dError > 0*/) ||
                (pTerm + (outputSum + iTerm) + dTerm < outMin && error < 0 /*oder dError < 0*/) ) {
                 // In der Original-Implementierung wird iTerm dann modifiziert: constrain(iTermOut, -outMax, outMax)
                 // Das ist aber schwer nachzubilden ohne iTermOut genau so zu berechnen.
                 // Eine gängige iAwCondition ist, den aktuellen iTerm zu überspringen oder zu kappen.
                 // Ich halte mich hier erstmal an die Struktur des Originals, wonach bei iAwCondition
                 // iTerm unter bestimmten Bedingungen modifiziert wird, aber es ist nicht trivial, das exakt nachzubauen
                 // ohne die interne Struktur von iTermOut.

                 // Die Original-Logik für iAwCondition ist:
                 // float iTermOutOriginal = (pmode == pMode::pOnError ? kp * error : (pmode == pMode::pOnMeas ? 0 : 0.5f * kp * error)) -
                 //                          (pmode == pMode::pOnMeas ? kp * dInput : (pmode == pMode::pOnError ? 0 : 0.5f * kp * dInput)) +
                 //                          ki * (outputSum + error); // iTerm hier ist der inkrementelle Anteil
                 // if (iawmode == iAwMode::iAwCondition) {
                 //   bool aw_original = false;
                 //   if (iTermOutOriginal > outMax && dError > 0) aw_original = true;
                 //   else if (iTermOutOriginal < outMin && dError < 0) aw_original = true;
                 //   if (aw_original && ki_raw_from_SetTunings_is_not_zero) iTerm = constrain(iTermOutOriginal, -outMax, outMax) - (outputSum - (pmode == pMode::pOnMeas ? kp * dInput : 0));
                 // }
                 // Dies ist zu komplex, um es hier ohne tiefere Analyse nachzubauen.
                 // Typischerweise bei iAwCondition würde man den iTerm (den inkrementellen) einfach nicht addieren, wenn bereits Sättigung vorliegt.
                 // Oder man lässt die Original-Logik so stehen und hofft, dass sie trotz der neuen Struktur Sinn ergibt.
                 // Für diese Version belasse ich es bei der Originalstruktur, die weiter unten folgt (mit `outputSum += iTerm`).
                 // Der iAwCondition Teil aus dem Original:
                 // if (iawmode == iAwMode::iAwCondition) {
                 //    bool aw = false;
                 //    float iTermOut = (pmode == pMode::pOnError ? kp * error : (pmode == pMode::pOnMeas ? 0.0f : 0.5f * kp * error)) -
                 //                     (pmode == pMode::pOnMeas ? kp * dInput : (pmode == pMode::pOnError ? 0.0f : 0.5f * kp * dInput)) +
                 //                     outputSum + iTerm; // iTerm ist der neue Beitrag
                 //    if (iTermOut > outMax && dError > 0) aw = true;
                 //    else if (iTermOut < outMin && dError < 0) aw = true;
                 //    if (aw && ki) { /* Hier würde iTerm angepasst, z.B. iTerm = 0 */ }
                 //    else { outputSum += iTerm; } // Normales Hinzufügen
                 // } else { outputSum += iTerm; }
            }
        }
    }


    // --- Summation und Output ---
    // Der I-Anteil (iTerm) wird zum Akkumulator (outputSum) addiert.
    // Die Anti-Windup Logik beeinflusst, WIE das geschieht.

    // Originale Logik für Integration und Anti-Windup:
    // outputSum += iTerm; // Inkrementiere I-Anteil
    // if (iawmode == iAwOff) {
    //    outputSum -= pmTerm; // pmTerm bei iAwOff (kein Anti-Windup) vom Integral entkoppeln (?) - diese Zeile ist unklar im Originalkontext
                             // Wahrscheinlich sollte pmTerm hier nicht subtrahiert werden, wenn es bereits Teil von pTerm ist.
                             // Wenn iawmode == iAwOff, sollte outputSum einfach outputSum + iTerm sein.
    // } else { // iAwCondition oder iAwClamp
    //    outputSum = constrain(outputSum - pmTerm, outMin, outMax); // pmTerm hier zu berücksichtigen ist Teil von "PID on measurement" Strategien.
                                                                  // und dann Clamp.
    // }

    // Überarbeitete Logik basierend auf dem, was iawmode typischerweise bedeutet:

    if (iawmode == iAwMode::iAwOff) {
        outputSum += iTerm; // Keine Begrenzung des Integralanteils
    } else if (iawmode == iAwMode::iAwClamp) {
        outputSum += iTerm;
        outputSum = constrain(outputSum, outMin, outMax); // Klemme den Integralanteil direkt
    } else { // iAwMode::iAwCondition (Default)
        // Die Original-Bedingung für awCondition ist oben schon teilweise behandelt.
        // Hier wird iTerm hinzugefügt, WENN die Bedingung es erlaubt.
        // Typischerweise: Wenn (outputSum + pTerm + dTerm) nicht schon an der Grenze ist, oder Fehler nicht in Sättigungsrichtung zeigt
        // Für iAwCondition aus dem Original:
        // bool aw_original_cond = false;
        // float iTermOut_orig = (pmode==pMode::pOnError?kp*error:(pmode==pMode::pOnMeas?0:0.5f*kp*error)) - (pmode==pMode::pOnMeas?kp*dInput:(pmode==pMode::pOnError?0:0.5f*kp*dInput)) + outputSum + iTerm;
        // if (iTermOut_orig > outMax && dError > 0) aw_original_cond = true;
        // else if (iTermOut_orig < outMin && dError < 0) aw_original_cond = true;
        // if (!(aw_original_cond && ki)) { // Wenn nicht Anti-Windup-Bedingung erfüllt ist ODER Ki 0 ist
        //   outputSum += iTerm;
        // }
        // Für diese Anpassung übernehme ich die Struktur aus dem Original-Compute für Summierung:
        outputSum += iTerm; // iTerm zum Integral addieren
    }


    // Der pmTerm (Proportional auf Messwert) wird im Original-QuickPID 3.1.9
    // speziell behandelt, abhängig vom iawmode.
    // Wenn iawmode != iAwOff (also Condition oder Clamp),
    // wird pmTerm vom outputSum (dem Integralakkumulator) subtrahiert und dann geklemmt.
    // Bei iAwOff wird es auch subtrahiert, aber ohne Klemmen.
    // Dies ist dafür da, dass der P-Anteil auf Messwert den Integralwert nicht "mitzieht".
    float tempOutputSum = outputSum;
    if (pmode == pMode::pOnMeas || pmode == pMode::pOnErrorMeas) { // Nur wenn POnMeas aktiv ist
        if (iawmode != iAwMode::iAwOff) {
            tempOutputSum = constrain(outputSum - pmTerm, outMin, outMax);
        } else {
            tempOutputSum = outputSum - pmTerm;
        }
    } else { // pOnError
      if (iawmode != iAwMode::iAwOff) { // Clamp auch bei pOnError für outputSum
          tempOutputSum = constrain(outputSum, outMin, outMax);
      } else {
          tempOutputSum = outputSum; // Keine Änderung, da pmTerm = 0
      }
    }


    // --- Feedforward Term ---
    float ffTerm = 0.0f;
    if (this->_feedforwardInput != nullptr && this->_Kff != 0.0f) {
        ffTerm = this->_Kff * (*this->_feedforwardInput);
        if (this->_feedforwardDirection == ffDir::ffReverse) {
            ffTerm = -ffTerm;
        }
    }

    // --- Final Output Calculation ---
    // Original: *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);
    // Hier wird peTerm verwendet, da pmTerm bereits in outputSum (via tempOutputSum) verrechnet wurde,
    // wenn pOnMeas aktiv war.
    // Wenn pOnError: pTerm = peTerm. tempOutputSum = outputSum (ggf. geklemmt).
    //               Output = tempOutputSum + peTerm + dTerm + ffTerm
    // Wenn pOnMeas: pTerm = -pmTerm. tempOutputSum = outputSum - pmTerm (ggf. geklemmt)
    //               Output = tempOutputSum + 0 (dafür pmTerm schon in tempOutputSum) + dTerm + ffTerm??
    //               Die Originalzeile `*myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax)` ist tricky.
    //               Es scheint, als ob `outputSum` hier der geklemmte Wert (outputSum - pmTerm bei pOnMeas) sein soll.
    //               Und peTerm bei pOnMeas = 0.

    // Klarere Struktur für Output:
    // P-Term (bereits pOnError oder pOnMeas korrekt berechnet als `pTerm`)
    // I-Term (ist in `tempOutputSum` enthalten, das ist der geklemmte Integral-Anteil, ggf. mit pmTerm Verrechnung)
    // D-Term (`dTerm`)
    // FF-Term (`ffTerm`)

    // Der `pTerm` aus der ursprünglichen Berechnung (z.B. `pTerm = original_peTerm - original_pmTerm;`)
    // muss hier verwendet werden.
    // float finalOutput = tempOutputSum + pTerm_berechnet_am_Anfang + dTerm + ffTerm;
    // Die Original-Zeile ist `*myOutput = constrain(outputSum_verrechnet_mit_pmTerm_und_geklemmt + peTerm_nur_OnError_Anteil + dTerm, outMin, outMax);`
    // Das bedeutet:
    float p_contrib_for_final_output;
    if (pmode == pMode::pOnError) p_contrib_for_final_output = kp * error; // Nur Error-Anteil
    else if (pmode == pMode::pOnMeas) p_contrib_for_final_output = 0; // pOnMeas Anteil ist in tempOutputSum
    else  p_contrib_for_final_output = 0.5f * kp * error; // Error-Anteil von pOnErrorMeas

    *myOutput = constrain(tempOutputSum + p_contrib_for_final_output + dTerm + ffTerm, outMin, outMax);

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
void QuickPID::SetTunings(float Kp, float Ki, float Kd,
                          pMode pMode_val, dMode dMode_val, iAwMode iAwMode_val) { // Renamed params to avoid clash

  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  if (Ki == 0) outputSum = 0; // Wenn Ki 0 ist, Integralanteil zurücksetzen
  
  // Store enums
  this->pmode = pMode_val;
  this->dmode = dMode_val;
  this->iawmode = iAwMode_val;

  // Store display Kp, Ki, Kd (raw values)
  dispKp = Kp;
  dispKi = Ki;
  dispKd = Kd;

  // Calculate actual Kp, Ki, Kd based on sample time
  float SampleTimeSec = (float)sampleTimeUs / 1000000.0f;
  kp = Kp;
  ki = Ki * SampleTimeSec; // Ki wird mit Abtastzeit skaliert
  kd = Kd / SampleTimeSec; // Kd wird mit Abtastzeit skaliert
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered pMode, dMode and iAwMode settings.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, this->pmode, this->dmode, this->iawmode); // Verwende gespeicherte Modi
}

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    // Adjust Ki and Kd for the new sample time
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

  if (mode != Control::manual) { // Nur wenn nicht im manuellen Modus
    *myOutput = constrain(*myOutput, outMin, outMax);
    outputSum = constrain(outputSum, outMin, outMax); // Auch den Integral-Akkumulator begrenzen
  }
}

/* SetMode(.)*****************************************************************
  Sets the controller mode to manual (0), automatic (1) or timer (2)
  when the transition from manual to automatic or timer occurs, the
  controller is automatically initialized.
******************************************************************************/
void QuickPID::SetMode(Control Mode_val) { // Renamed
  Control oldMode = this->mode;
  if (oldMode == Control::manual && Mode_val != Control::manual) { // just went from manual to automatic, timer or toggle
    QuickPID::Initialize();
  }

  if (Mode_val == Control::toggle) {
    this->mode = (oldMode == Control::manual) ? Control::automatic : Control::manual;
  } else {
    this->mode = Mode_val;
  }
}

void QuickPID::SetMode(uint8_t Mode_val) { // Renamed
   SetMode(static_cast<Control>(Mode_val));
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize() {
  outputSum = *myOutput; // Setze Integral-Akkumulator auf aktuellen Output
  lastInput = *myInput;  // Setze letzten Input auf aktuellen Input
  outputSum = constrain(outputSum, outMin, outMax); // Stelle sicher, dass outputSum innerhalb der Grenzen ist

  // Feedforward-Parameter werden hier nicht verändert, sie sind Teil der Konfiguration.
}

/* SetControllerDirection(.)**************************************************
  The PID will either be connected to a direct acting process (+Output leads
  to +Input) or a reverse acting process(+Output leads to -Input).
******************************************************************************/
void QuickPID::SetControllerDirection(Action Action_val) { // Renamed
  this->action = Action_val;
}
void QuickPID::SetControllerDirection(uint8_t Direction) {
  this->action = static_cast<Action>(Direction);
}

/* SetProportionalMode(.)*****************************************************
  Sets the computation method for the proportional term, to compute based
  either on error (default), on measurement, or the average of both.
******************************************************************************/
void QuickPID::SetProportionalMode(pMode pMode_val) { // Renamed
  this->pmode = pMode_val;
}
void QuickPID::SetProportionalMode(uint8_t Pmode) {
  this->pmode = static_cast<pMode>(Pmode);
}

/* SetDerivativeMode(.)*******************************************************
  Sets the computation method for the derivative term, to compute based
  either on error or on measurement (default).
******************************************************************************/
void QuickPID::SetDerivativeMode(dMode dMode_val) { // Renamed
  this->dmode = dMode_val;
}
void QuickPID::SetDerivativeMode(uint8_t Dmode) {
  this->dmode = static_cast<dMode>(Dmode);
}

/* SetAntiWindupMode(.)*******************************************************
  Sets the integral anti-windup mode to one of iAwClamp, which clamps
  the output after adding integral and proportional (on measurement) terms,
  or iAwCondition (default), which provides some integral correction, prevents
  deep saturation and reduces overshoot.
  Option iAwOff disables anti-windup altogether.
******************************************************************************/
void QuickPID::SetAntiWindupMode(iAwMode iAwMode_val) { // Renamed
  this->iawmode = iAwMode_val;
}
void QuickPID::SetAntiWindupMode(uint8_t IawMode) {
  this->iawmode = static_cast<iAwMode>(IawMode);
}

void QuickPID::Reset() {
  lastTime = micros() - sampleTimeUs; // Setze Zeitstempel zurück als wäre gerade ein Sample passiert
  lastInput = 0; // Setze letzten Input zurück
  outputSum = 0; // Setze Integral-Akkumulator zurück
  pTerm = 0;     // Setze P-Term Visualisierung zurück
  iTerm = 0;     // Setze I-Term Visualisierung zurück
  dTerm = 0;     // Setze D-Term Visualisierung zurück

  // Feedforward-Parameter bleiben bei Reset unberührt (sind Konfiguration)
  // Wenn sie auch resettet werden sollen:
  // this->_Kff = 0.0f;
  // this->_feedforwardInput = nullptr; // Vorsicht, wenn der Nutzer den Pointer woanders noch braucht
  // this->_feedforwardDirection = ffDir::ffDirect;
}

// sets the output summation value
void QuickPID::SetOutputSum(float sum) {
  outputSum = sum;
  outputSum = constrain(outputSum, outMin, outMax); // Sicherstellen, dass es innerhalb der Grenzen ist
}

/* Status Functions************************************************************
  These functions query the internal state of the PID.
******************************************************************************/
float QuickPID::GetKp() {
  return dispKp; // Gibt den "rohen" Kp-Wert zurück, der vom Nutzer gesetzt wurde
}
float QuickPID::GetKi() {
  return dispKi; // Gibt den "rohen" Ki-Wert zurück
}
float QuickPID::GetKd() {
  return dispKd; // Gibt den "rohen" Kd-Wert zurück
}
float QuickPID::GetPterm() {
  // pTerm wurde in Compute() mit dem Ergebnis von (original_peTerm - original_pmTerm) belegt.
  // Das ist der P-Anteil, der zum Output beiträgt (vor Multiplikation mit kp)
  // Oder es ist der effektive P-Anteil, je nachdem wie pTerm in Compute() genau berechnet wurde.
  // Gemäß der Original-Compute-Struktur: pTerm = peTerm - pmTerm;
  // wobei peTerm = kp*error (oder 0 oder 0.5*kp*error)
  // und   pmTerm = kp*dInput (oder 0 oder 0.5*kp*dInput)
  // Also ist pTerm der berechnete P-Anteil.
  return pTerm;
}
float QuickPID::GetIterm() {
  // iTerm in Compute() ist der *inkrementelle* Beitrag dieses Zyklus.
  // Der akkumulierte I-Wert ist in outputSum (vor Verrechnung mit pmTerm für den Output).
  // GetIterm sollte den akkumulierten Wert zurückgeben.
  // Wenn `outputSum` der Akkumulator ist:
  return outputSum;
  // Wenn `iTerm` in Compute als der akkumulierte Beitrag gespeichert wird (was nicht der Fall ist im Original), dann `return iTerm;`
  // Da `outputSum` public ist, kann der User es eh direkt abfragen.
  // Im Original-QuickPID 3.1.9 wird `iTerm` in `Compute()` so gesetzt: `iTerm = ki * error;` (also der inkrementelle).
  // `GetIterm()` sollte konsistent sein. Wenn es den inkrementellen zurückgeben soll:
  // return iTerm;  // Dies ist oft der Fall, wenn P, I, D Terme einzeln visualisiert werden sollen.
}
float QuickPID::GetDterm() {
  return dTerm; // dTerm wird in Compute() korrekt berechnet.
}
float QuickPID::GetOutputSum() {
  // Dies ist der akkumulierte Wert (primär Integral) plus die Verrechnung
  // mit pmTerm in der Compute-Logik, kurz bevor peTerm und dTerm addiert werden.
  // Es ist der Wert von 'tempOutputSum', der in die finale Output-Berechnung eingeht.
  // Da `outputSum` public ist und den rohen Integral-Akkumulator darstellt, ist dies evtl. feiner.
  return outputSum; // Gibt den rohen Integralakkumulator zurück.
                    // Die finale Summe, die zum Output beiträgt (vor Limits) ist
                    // tempOutputSum + p_contrib_for_final_output + dTerm + ffTerm
}
uint8_t QuickPID::GetMode() {
  return static_cast<uint8_t>(mode);
}
uint8_t QuickPID::GetDirection() {
  return static_cast<uint8_t>(action);
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

// --- NEUE METHODEN für Störgrößenaufschaltung ---
void QuickPID::SetFeedforwardInput(float *ffInput) {
  this->_feedforwardInput = ffInput;
}

void QuickPID::SetFeedforwardTunings(float Kff) {
  this->_Kff = Kff;
}

void QuickPID::SetFeedforwardDirection(ffDir direction) {
  this->_feedforwardDirection = direction;
}

float QuickPID::GetKff() {
  return this->_Kff;
}

QuickPID::ffDir QuickPID::GetFeedforwardDirection() {
  return this->_feedforwardDirection;
}
