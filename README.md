# QuickPID Arduino Library (Erweitert mit Störgrößenaufschaltung / Feedforward Control)

Dies ist eine erweiterte Version der robusten [QuickPID Arduino Library](https://github.com/Dlloydev/QuickPID) von Dlloydev. Diese Version integriert eine Funktion zur **Störgrößenaufschaltung (Feedforward Control)** direkt in die Kernfunktionalität der Bibliothek, um die Regelleistung bei bekannten oder messbaren Störeinflüssen zu verbessern.

## Original QuickPID

Die ursprüngliche QuickPID-Bibliothek bietet eine schnelle, effiziente und flexible PID-Regelung für Arduino-Projekte. Sie zeichnet sich aus durch:

*   Verschiedene Proportional- (POnError, POnMeas), Integral- (Anti-Windup Modi) und Differential-Modi (DOnError, DOnMeas).
*   Einstellbare Abtastzeiten.
*   Manuelle und automatische Betriebsmodi.
*   Möglichkeit, Reglerparameter zur Laufzeit zu ändern.
*   Unterstützung für direkte und umgekehrte Reglerwirkung.
*   Einen "Offset"- oder "Bias"-Wert, der zum Ausgang addiert werden kann.

**Für eine vollständige Dokumentation der Basisfunktionen siehe das [Original-Repository](https://github.com/Dlloydev/QuickPID).**

## Neue Funktion: Störgrößenaufschaltung (Feedforward Control)

Diese Erweiterung fügt eine proportionale Störgrößenaufschaltung hinzu. Damit kann der Regler proaktiv auf bekannte Störungen reagieren, bevor diese die Regelgröße signifikant beeinflussen. Dies führt oft zu einer schnelleren Ausregelung und geringeren Regelabweichungen.

### Kernmerkmale der Erweiterung:

*   **Proportionaler Feedforward-Faktor (Kff):** Ein Faktor, der die Stärke der Aufschaltung definiert.
*   **Störgrößen-Input:** Ein Pointer auf eine Variable, die den aktuellen Wert der gemessenen Störgröße enthält.
*   **Feedforward-Richtung:** Definiert, ob der Feedforward-Term zum PID-Ausgang addiert (`ffDirect`) oder davon subtrahiert (`ffReverse`) werden soll. Dies ermöglicht eine flexible Anpassung an verschiedene Prozess- und Störungscharakteristiken.

### Wie es funktioniert:

1.  Der PID-Regler berechnet seinen Anteil (P, I, D) wie gewohnt.
2.  Parallel dazu wird ein Feedforward-Term berechnet: `ffTerm = Kff * Störgrößen_Input_Wert`.
3.  Abhängig von der eingestellten `Feedforward-Richtung` wird dieser `ffTerm` (ggf. negiert) zum Bias und den PID-Termen addiert, bevor die Ausgangsgrenzen angewendet werden.
    `Gesamtausgang = pTerm + iTerm + dTerm + Bias + ffTerm_angepasst`
4.  Wenn `Kff` auf `0` gesetzt ist oder kein `Störgrößen_Input` konfiguriert wurde, ist die Störgrößenaufschaltung effektiv deaktiviert und der `ffTerm` ist `0`.

### Neue Methoden:

*   `void SetFeedforwardInput(float *ffInput)`:
    Übergibt einen Pointer auf die Variable, die den Wert der Störgröße enthält.
    *   `ffInput`: Pointer zur float-Variablen mit dem Störgrößenwert.

*   `void SetFeedforwardTunings(float Kff)`:
    Setzt den Verstärkungsfaktor für die Störgrößenaufschaltung.
    *   `Kff`: Der proportionale Feedforward-Faktor. Kann positiv oder negativ sein, üblicherweise wird er aber positiv gehalten und die Richtung separat gesteuert.

*   `void SetFeedforwardDirection(QuickPID::ffDir direction)`:
    Bestimmt, wie der Feedforward-Term auf den Ausgang wirkt.
    *   `direction`: `QuickPID::ffDir::ffDirect` (Standard) oder `QuickPID::ffDir::ffReverse`.

*   `float GetKff()`:
    Gibt den aktuell eingestellten Kff-Wert zurück.

*   `QuickPID::ffDir GetFeedforwardDirection()`:
    Gibt die aktuell eingestellte Feedforward-Richtung zurück.

### Beispielhafte Anwendung:

```cpp
#include <QuickPID_Kff.h>

// Sensoren, Aktoren & Störgröße
float temperaturIst;
float heizleistungAusgang;
float sollTemperatur = 22.0;
float aussentemperaturStoerung; // Unsere messbare Störgröße

// PID & Feedforward Parameter
float Kp = 2.0, Ki = 0.5, Kd = 0.1;
float Kff = 0.8; // Faktor für die Störgrößenaufschaltung

QuickPID myPID(&temperaturIst, &heizleistungAusgang, &sollTemperatur,
               Kp, Ki, Kd, QuickPID::pMode::pOnError, QuickPID::cDir::dDirect, 100);

void setup() {
  Serial.begin(115200);

  myPID.SetOutputLimits(0, 255); // z.B. für PWM
  myPID.SetMode(QuickPID::cMode::automatique);

  // Konfiguration der Störgrößenaufschaltung
  myPID.SetFeedforwardInput(&aussentemperaturStoerung);
  myPID.SetFeedforwardTunings(Kff);
  // Annahme: Höhere Außentemperatur -> weniger Heizleistung nötig (reverse action für den Feedforward)
  // Wenn Kff positiv ist, wird der FF-Term bei steigender Außentemperatur größer.
  // Wir wollen dann aber den Output verringern, also Reverse.
  myPID.SetFeedforwardDirection(QuickPID::ffDir::ffReverse); 
  // Alternativ: Kff negativ setzen und Richtung Direct lassen,
  // oder die aussentemperaturStoerung so transformieren, dass sie direkt die "benötigte Kompensation" darstellt.
}

void loop() {
  // Istwerte und Störgröße einlesen
  temperaturIst = readIstTemperatur();
  aussentemperaturStoerung = readAussentemperatur(); // z.B. 10.0 für 10°C

  if (myPID.Compute()) {
    // heizleistungAusgang wurde aktualisiert
    analogWrite(HEATER_PIN, (int)heizleistungAusgang);

    Serial.print("Soll: "); Serial.print(sollTemperatur);
    Serial.print(" Ist: "); Serial.print(temperaturIst);
    Serial.print(" StörG: "); Serial.print(aussentemperaturStoerung);
    Serial.print(" Out: "); Serial.println(heizleistungAusgang);
  }
  delay(100); // PID-Abtastintervall
}

float readIstTemperatur() { /* ... Ihre Sensorlogik ... */ return 20.0; }
float readAussentemperatur() { /* ... Ihre Sensorlogik für Störgröße ... */ return 15.0; }
