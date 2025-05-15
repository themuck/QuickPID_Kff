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
