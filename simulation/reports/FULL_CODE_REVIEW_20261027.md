# Firmware-Codeprüfung vom 27.10.2026

**Nachfolgende Korrekturen:** Die unten dokumentierten belegten Fehler wurden
anschliessend bearbeitet. Den aktuellen Stand mit erweiterten Prüfresultaten
beschreibt [RUNTIME_AND_ROUTE_FIXES_20261027.md](RUNTIME_AND_ROUTE_FIXES_20261027.md).
Die folgenden Messwerte und Offen-Befunde beschreiben den ursprünglichen
Prüfstand und bleiben als Fehlernachweis erhalten.

Prüfstand: HEAD `861ed8e` plus die bereits vorbereiteten Änderungen an der
Innenlinie für Runde 2/3. In dieser Prüfung wurde **keine Firmware geändert,
kompiliert oder aufgespielt**. Hinzugefügt wurden reproduzierbare Prüfskripte und
dieser Befund. Die Fahrversion hat noch offene Fehler; dies ist keine Freigabe.

## Ergebnis und Prioritäten

Es wurden konkrete Funktionsfehler gefunden. Die grössten Probleme liegen bei
Wiederanlauf, zeitlichem Ablauf und der Absicherung der jüngsten Wegoptimierung.
Die vorhandenen Logs rechtfertigen keine pauschale Erklärung durch eine defekte
Kamera. Hinweise auf absichtlich schädliche Funktionen wurden im geprüften
Projektcode nicht gefunden.

| Priorität | Befund | Nachweis |
| --- | --- | --- |
| Hoch | Neueste Innenlinie verletzt in sechs zusätzlichen Modellfällen bestehende Reserven | Vollständiger A/B-Vergleich der 2191 vorhandenen ungedrosselten Testfälle mit echter Motorprofilrampe |
| Hoch | Nach Gyro-Halt können Rückwärtsbeobachtung und Open-Navigation mit gesperrten Lenkbefehlen weiterfahren | Originale Aktuatorfunktionen und akzeptierter Reverse-Befehlszweig ausgeführt |
| Hoch | Grosser Schleifenabstand wird vollständig als neue Beschleunigungs- und Sollwegzeit integriert | Originales `drive_loop()` mit 20–1200 ms ausgeführt |
| Hoch | Ein neuer Fahrversuch kann während USB-Speicherung starten; Dateisuche ist innerhalb eines Updates unbeschränkt | Originales `mode_resume()` und Dateisuchblock ausgeführt |
| Mittel | M4-I2C meldet auch nach einem Datenbit-Timeout die volle angeforderte Bytezahl | Originale `readByte()` / `requestFrom()` mit injiziertem SCL-Timeout ausgeführt |
| Mittel | Open-Navigation zählt Schleifendurchläufe statt neuer ToF-Messungen zur Eckbestätigung | Direkte Prüfung der Zustandslogik |

„Hoch“ bedeutet hier: vor einer allgemeinen Fahrfreigabe gezielt bearbeiten.
Es bedeutet nicht, dass jeder normale Lauf den Fehler auslöst. Die
hardwareabhängigen Adapter in den Host-Prüfungen sind ausdrücklich simuliert;
die untersuchten Entscheidungs- und Aktuatorfunktionen stammen aus dem Projekt.

## 1. Neueste Innenlinie: geometrischer PASS reicht nicht

Quellen: [shortenKnownInnerRoute und Übernahme](../../src/obstacle_path.cpp),
`laterLapTrackedRouteSafe()` ab Zeile 2925, `laterLapTrackingSpeed()` ab 2835,
`drive_loop()` in [motor_control.cpp](../../src/motor_control.cpp) ab 678.

Die bisherige Geometriematrix wurde frisch aus dem aktuellen Code erzeugt:
**2807/2807 bestanden**. Auch die bisherigen sieben Beispielszenen bei
10/20/50 ms bestehen erneut, also 21 dynamische Beispiele. Diese Aussagen bleiben
korrekt, decken aber die Randfälle nicht ab.

Die Erweiterung auf **alle 2191 vorhandenen Fälle ohne zusätzliche
Geschwindigkeitsdrosselung bei 20 ms** ergibt:

- aktuelle Innenlinien-Optimierung: **2184 bestanden, 7 verletzt**;
- identischer Code mit ausschliesslich dem neuen Aufruf
  `shortenKnownInnerRoute(optimizedPath,beforeLaneCarry)` entfernt:
  **2190 bestanden, 1 verletzt**;
- damit **sechs zusätzliche Grenzverletzungen durch die neue Optimierung**;
  der siebte Fall bestand bereits vorher.

| Fall-ID der ungekappten Teilmatrix | Richtung | Hindernisreserve neu / Kontrolle, mm | Maximale Querbeschleunigung neu / Kontrolle, m/s² |
| --- | --- | --- | --- |
| 116 | CW | 29.5314 / 37.5838 | 0.603580 / 0.552878 |
| 129 | CW | 28.6248 / 37.0587 | 0.602594 / 0.597267 |
| 139 | CW | 28.2510 / 36.9869 | 0.596284 / 0.590211 |
| 154 | CW | 48.6886 / 48.6886 | **0.803892 / 0.803892** |
| 395 | CCW | 29.0085 / 35.9484 | 0.625564 / 0.547517 |
| 399 | CCW | 29.5479 / 37.5517 | 0.603580 / 0.555679 |
| 548 | CCW | 72.0490 / 77.9958 | **0.808221 / 0.574036** |

Die verlangten Grenzen sind mindestens **30 mm** modellierter Hindernisabstand
und höchstens **0.8 m/s²** Querbeschleunigung. Fünf neue Fälle unterschreiten
den Abstand um 0.45–1.75 mm, einer überschreitet das Beschleunigungsbudget um
etwa 1.03 %. Das sind **keine modellierten Kollisionen**, aber echte Verletzungen
der gewählten Akzeptanzkriterien. Eine physische Sicherheitsbehauptung lässt sich
daraus nicht ableiten.

Ursache der Prüflücke: Die optionale Firmware-Freigabe verwendet einen festen
räumlichen Schritt von 11.44 mm und eine eigene Zielpunktsuche. Der erweiterte
Test verwendet die vorhandene Fahrnachbildung mit wirklicher Motorprofilrampe,
zeitabhängigen Schritten und deren Zielpunktsuche. Geschwindigkeitsbegrenzung
als Sollwert erzwingt wegen der endlichen Verzögerung nicht sofort die
entsprechende Istgeschwindigkeit. Im bereits vorher auffälligen Fall 154
erreicht das Verhältnis Profilgeschwindigkeit zu momentaner Anforderung 1.4593.
Eine konstante Schrittweite plus Toleranz ersetzt diese Dynamikprüfung nicht.

Die neue Suche hat auf Hardware ein kooperatives 30-ms-Budget; im Hostmodell
steht `micros()` still und alle Kandidaten werden ausgewertet. Auf dem Roboter
kann deshalb häufiger die alte Linie bleiben. Das macht die oben gefundenen
Kandidaten nicht ausreichend abgesichert und beweist nicht, dass ein bestimmter
Kandidat bereits physisch gefahren wurde.

Nächste Korrektur: Kandidatenprüfung und tatsächliche Zielpunkt-/Rampenlogik
angleichen; problematische Kandidaten auf die geprüfte vorherige Form
zurückfallen lassen. Den bereits vorher auffälligen Fall 154 separat durch
frühere gezielte Verzögerung behandeln. Keine pauschale Verlangsamung aller
Geraden und keine zusätzlichen Entdeckungsstopps daraus ableiten.

### Reproduzierbare betroffene Karten

Arrayindex = kanonischer Sitz 0–23; 0 leer, 1 GREEN, 2 RED. Die IDs beziehen
sich auf die nach `speed_cap == 0` gefilterte aktuelle Matrix, nicht auf Logs.

```text
116 CW  [0,0,2,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0]
129 CW  [0,0,2,0,0,0,1,0,0,0,0,2,1,0,0,0,0,2,1,0,0,0,0,2]
139 CW  [0,0,2,0,0,0,0,2,0,0,0,2,0,2,0,0,0,2,0,2,0,0,0,2]
154 CW  [0,0,0,0,1,0,2,0,0,0,1,0,2,0,0,0,1,0,2,0,0,0,1,0]
395 CCW [0,0,0,1,0,0,0,2,0,0,0,0,0,2,0,0,0,0,0,2,0,0,0,0]
399 CCW [0,0,0,1,0,0,0,0,0,2,0,0,0,0,0,2,0,0,0,0,0,2,0,0]
548 CCW [0,1,0,0,0,2,1,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,1]
```

## 2. Lenkungsfreigabe nach Sensorpause ist noch unvollständig

Quellen: [main.cpp](../../src/main.cpp), Gyro-Halt;
[motor_control.cpp](../../src/motor_control.cpp), `stop()` ab 770, `steer()` ab 270,
`set_speed()` ab 806; [obstacle_path.cpp](../../src/obstacle_path.cpp),
`CORNER_VIEW_REVERSE` ab 4389 und `CORNER_VIEW_RETURN` ab 4479;
[navigation_controller.cpp](../../src/navigation_controller.cpp),
`state_following()` ab 186 und `state_turning()` ab 346.

`stop(false)` setzt `servo_disabled=true`. `set_speed()` aktiviert danach den
Antrieb, hebt die Lenksperre aber nicht auf. Die normalen Pfadbefehle und die
finalen Parksegmente wurden bereits korrigiert; die genannten Zweige der
Beobachtungsrückfahrt sowie die Open-Navigation noch nicht.

Nachbildung mit Originalfunktionen: nach Halt und akzeptiertem Reverse-Befehl
Motor aktiv, Lenksperre aktiv, angefordert 7°, ausgegeben weiterhin 0°.
Mit Wiederfreigabe vor demselben akzeptierten Befehl werden 7° ausgegeben.
Bei einem zuvor anderen Lenkwinkel bleibt entsprechend dieser alte Winkel
stehen. Die Prüfung setzt sichere Geometrie voraus; sie untersucht die
Aktuatorübernahme, nicht eine echte Sensoraufnahme.

Zusätzlich laufen die `cornerViewPhaseMs`-Zeitgrenzen während einer Gyro-Pause
weiter; anders als beim finalen Parkcontroller wird diese Pause nicht abgezogen.
Eine lange Sensorpause kann daher einen eigentlich noch offenen Bewegungsschritt
in einen permanenten Zeitlimit-Halt verwandeln. Kein Nachweis, dass genau dies
die fehlenden neuesten CW-Läufe erklärt.

Nächste Korrektur: alle akzeptierten Fahrbefehlszweige systematisch mit
gezielt injizierter Sensorpause prüfen und die Freigabe erst nach erfolgreicher
Gesundheits-/Geometrieprüfung herstellen. Sensorfehler und verworfene Befehle
müssen weiterhin bremsen.

## 3. Lange Schleifenpause erzeugt künstlichen Sollweg

Quelle: [motor_control.cpp](../../src/motor_control.cpp), `loop_updater()` ab 827,
`drive_loop()` ab 678, Sollwegintegration Zeile 755.

Der volle zeitliche Abstand des letzten Hauptschleifendurchlaufs wird für
Beschleunigungsrampe und Sollweg verwendet. Eine neu freigegebene Fahrt kann
damit so behandelt werden, als hätte sie während einer vorherigen blockierten
oder angehaltenen Zeit bereits beschleunigt.

Originalcode, Ausgangsgeschwindigkeit/Beschleunigung/Sollweg null,
neue Anforderung 260 mm/s:

| Ein einzelner Zeitschritt | Neue Profilgeschwindigkeit, mm/s | Neuer Sollweg, mm |
| --- | --- | --- |
| 20 ms | 0.8 | 0.016 |
| 50 ms | 5.0 | 0.250 |
| 215 ms | 72.67 | 15.6241 |
| 819 ms | 260.0 | 212.94 |
| 1200 ms | 260.0 | 312.00 |

212.94 mm sind ein Sprung der **Regler-Sollposition**, keine gemessene Fahrt.
Der Positionsregler kann diesen künstlichen Abstand anschliessend nachholen
wollen. In den archivierten Logs stehen tatsächlich grosse Pollinglücken;
die konkrete Kombination aus Rampenzustand und Sollweg wurde dort jedoch nicht
vollständig protokolliert. Der Fehler ist im Code reproduziert, seine genaue
Beteiligung an jedem physischen Unfall ist nicht bewiesen.

Nächste Korrektur: eine explizite Wiederanlauf-/Zeitlückenbehandlung, die
gemessene Bewegung berücksichtigt und keinen rückwirkenden neuen Sollweg
erfindet. Lange Rechen-/USB-Arbeit muss ausserdem aus dem bewegten Regelablauf
verschwinden. Nur ein blindes Begrenzen von `dt` würde nicht alle Probleme lösen.

## 4. USB-Speicherung und Neustart können sich überlappen

Quellen: [mode_manager.cpp](../../src/mode_manager.cpp), `mode_resume()` ab 520
und Modewechsel ab 251; [logger.cpp](../../src/logger.cpp), `update()` ab 269,
`LOGGER_OPEN_FILE` ab 339 und `LOGGER_WRITE` ab 369.

`mode_resume()` startet den angeforderten Modus ohne zu prüfen, ob der vorige
Speichervorgang noch läuft. Reproduktion: `logger_busy=1`, System danach
aktiv und `start_mode()` ausgeführt. Der Logger wird am Schleifenende auch
während der Fahrt weiter bedient.

Der Zustandsautomat verteilt zwar Phasen auf mehrere Updates; einzelne Aufrufe
von Mount/Open/Write/Flush bleiben synchron. Besonders deutlich: Die Suche
nach dem nächsten Dateinamen enthält ein unbeschränktes `do ... while(true)`.
Mit 534 simulierten vorhandenen Dateien werden in **einem** Update 535
Dateiöffnungsversuche ausgeführt. Das ist eine demonstrierte Operationszahl,
keine auf dem echten Stick gemessene Laufzeit.

Damit ist die Bezeichnung „non-blocking“ nur eingeschränkt zutreffend. Schneller
Neustart während alter Speicherung kann neue Gyro-/Reglerlücken verursachen.
Das ist ein möglicher Verstärker, kein belegter alleiniger Auslöser aller Logs.

Nächste Korrektur: laufende USB-Dateiarbeit konsequent mit stehenden Aktuatoren
koordinieren und Startwünsche eindeutig behandeln; Dateinamensuche begrenzen
oder inkrementell ausführen. Nicht während einer bestehenden Fahrt plötzlich
eine neue globale Fehlersperre einführen. Die vorhandene vollständige
RAM-Aufbewahrung bis zur erfolgreichen Dateiprüfung ist sinnvoll und bleibt.

## 5. M4-I2C verschluckt Lesefehler

Quelle: [software_i2c.cpp](../../src/m4/software_i2c.cpp), `readByte()` ab 122,
`requestFrom()` ab 179.

Scheitert `waitSclHigh()` in einer Datenbitphase, bricht `readByte()` nur aus der
Bitschleife aus und liefert ein teilweise gelesenes Byte. Auch das Ergebnis
der nachfolgenden ACK-Taktprüfung wird ignoriert. `requestFrom()` zählt die
Bytes trotzdem als vollständig empfangen.

Originalcode mit erfolgreicher Adressierung und danach blockiertem SCL:
2 Bytes angefordert, **2 Bytes als empfangen/lesbar gemeldet**, obwohl vier
Taktwartefehler auftraten. Die darüberliegende Bibliothek bekommt damit keinen
zuverlässigen Kurzlese-/Busfehler. Falsche oder unbrauchbare hintere
ToF-Daten sind unter dieser Bedingung möglich; ein tatsächlich fehlerhafter
Messwert in den letzten Läufen ist damit nicht nachgewiesen.

Nächste Korrektur: Fehler bis zum Aufrufer weiterreichen, kein teilweise
gelesenes Byte als vollständig melden; Daten- und ACK-Timeout separat prüfen.
Eine solche M4-Änderung erfordert einen M4-Build. M7 nur bei tatsächlich
mitgeänderten Schnittstellen/Verbrauchern zusätzlich bauen.

## 6. Eckbestätigung der Open-Navigation zählt keine unabhängigen Messungen

Quelle: [navigation_controller.cpp](../../src/navigation_controller.cpp),
`state_following()` Zeilen 224–241.

`nav_corner_gap_samples` steigt bei jedem Schleifendurchlauf mit derselben
zwischengespeicherten Distanz. Vier Zählschritte können daher viermal dieselbe
ToF-Messung bedeuten. Das ist keine Bestätigung durch mehrere unabhängige Frames.
ToF-Aktualität wird an anderer Stelle geprüft, ersetzt aber keine neue Sequenz.

Nächste Korrektur: pro relevanter neuer ToF-Sequenz höchstens einmal bestätigen.
Das betrifft direkt Open Challenge/den alten Navigationspfad. Der heutige
räumliche Obstacle-Pfad ist dadurch nicht automatisch die Ursache der jüngsten
CW-Ausfälle.

## Weitere Risiken mit noch fehlendem Laufzeitnachweis

**Live-Kamerapuffer:**
[arducam_dvp.cpp](../../lib/arducam_dvp_async/src/arducam_dvp.cpp),
`acquireContinuousFrame()` ab 926, liefert einen Zeiger auf den abgeschlossenen
DMA-Doppelpuffer. IRQ-Schutz sichert nur Index/Sequenz beim Lesen, nicht den
anschliessenden Bildinhalt. Nach dem nächsten Pufferwechsel kann DMA diesen
Speicher wieder beschreiben. Vision und die spätere lokale GREEN-Nachsuche
arbeiten direkt darauf, ohne Besitzübergabe oder abschliessende
Gültigkeitsprüfung. Bei rechtzeitigem kurzen Lesen ist das unauffällig;
späte Aufnahme nach langer Schleife oder längere Verarbeitung kann einen
zeitlich gemischten Frame ermöglichen. **Quellcode-Risiko, hier nicht als echte
Bildkorruption reproduziert.** Die früheren Bildversätze hatten bereits eine
separat untersuchte Phasenursache. Stationäre Snapshot-Exporte pausieren DMA
und beweisen deshalb nicht automatisch die Stabilität des Live-Lesefensters.

**Gyro-Frische nach Stau:**
[sensors.cpp](../../src/sensors.cpp), `update_gyro()` ab 91, setzt das Messalter
beim Empfang eines gültigen Quaternion auf `millis()`. Der vorhandene
SH2-Messzeitstempel wird dabei nicht ausgewertet. Ein alter, erst später aus
dem Transport gelesener Bericht könnte deshalb als frisch gelten. Ob/wie viele
solche Berichte die konkrete Sensor-/Treiberkombination nach einer Pause
puffert, ist hier nicht bewiesen. Erst Zeitbasis und tatsächlichen Rückstau
prüfen; nicht ungeprüft zwei verschiedene Zeitbasen subtrahieren.

**Initialisierungsfehler bleiben stehen:** M7-ToF/Gyro-Setup enthält
Endlosschleifen nach Initialisierungsfehlern. Auf M4 wird `INIT_FAILED` nach
einem fehlgeschlagenen Start nicht erneut initialisiert. Das ist ein
nachvollziehbarer sicherer Startabbruch, aber keine selbstheilende Behandlung
eines vorübergehenden Startfehlers. Es erklärt keine erst nach mehreren
erfolgreichen Abschnitten auftretende Störung.

## Abgleich mit den jüngsten tatsächlich vorhandenen Logs

Neueste archivierte Dateien sind 525–530. Ihre SHA-256-Werte wurden erneut mit
der [Evidenzliste](../evidence/parking_exit_diagnostics/README.md) verglichen und
stimmen überein. `log_529` ist ausdrücklich ein unvollständiger Ausschnitt.
Auf dem Rechner war bei der Prüfung kein Wechseldatenträger verfügbar.
Die vier danach berichteten CW-Läufe sind weiterhin nicht vorhanden und
wurden nicht nachträglich analysiert oder erfunden.

- **525–527 CW:** GREEN wird bestätigt; anschliessend Pollinglücken
  819/812/814 ms und später Stillstand unter Fahranforderung. Dazu existiert
  bereits eine Korrektur des normalen Pfad-Wiederanlaufs und der Rechenarbeit.
  Der hier gefundene noch offene Reverse-/Open-Zweig ist nicht mit dem schon
  korrigierten normalen Pfadzweig gleichzusetzen.
- **528/530 CCW:** GREEN 5 wird bestätigt. Danach scheitert der Connector
  mit 135.68°/144.33° Roboterheading und angeforderten 43.247°/42.007° über
  dem 42°-Trackinglimit. In 530 bestätigt die lokale Suche drei Frames mit
  x=85, Fuss=184, Range=267 mm und Snapfehler=23 mm. „GREEN wurde nicht
  erkannt“ passt zu diesen beiden Originalen nicht.
- **529 CCW:** Primär-/Wiederholungsblick bleiben UNKNOWN ohne geeignete
  Silhouette. Ein Erkennungs-/Sichtproblem ist hier tatsächlich vorhanden;
  ein vollständiges Fehlerbild oder Laufende fehlt.
- **Neu berichtetes finales Einparken:** Ohne die fehlenden Dateien keine
  belastbare Zuordnung zu `dual_marker_scan_incomplete`, Registrierungsfehler,
  Fahrvorprüfung oder einer späteren Segmentbedingung.

Die neu gefundene Runde-2/3-Regression erklärt die früheren Runde-1-Fehler
nicht: Die Optimierung ist dort nicht aktiv. Auch Zeitstempel einzelner
Übersetzungseinheiten beweisen wegen inkrementeller Builds keine komplette
Firmwareidentität. Gebautes Image und tatsächlich vom Benutzer installiertes
Image müssen auseinandergehalten werden.

## Unnötiges und bereits sinnvolle Schutzmassnahmen

Aufräumkandidaten, ohne sie in dieser Prüfung zu entfernen:

- `updateLearnedLanePlan()` und `updateCourseProgress()` in `obstacle.cpp`
  sind statische Alt-Funktionen ohne Aufrufstellen. Sie erklären kein aktuelles
  Fahrverhalten; vor Entfernung abhängige alte Felder/Diagnosen mitprüfen.
- `estimate_dc()` hat im Firmwareprojekt keine Aufrufstelle.
- `OBSTACLE_FINAL_PARKING_SCAN_ARM_X_MM` wird nicht mehr gelesen.
- `src/livecam` enthält alten Python-Kommentarcode und wird nicht als C++-Datei
  gebaut. Er verursacht keine Fahrverzögerung, kann aber beim Suchen verwirren.
- Der Kommentar in `logger.h`, `clear()` werde bei jedem `system_enable()`
  aufgerufen, stimmt nicht mit der Implementierung überein. Aktuell erfolgt
  das Leeren im Obstacle-Start über den historischen Exit-Test-Schalter.
- `OBSTACLE_CAMERA_INTERVAL_MS=50` steuert nicht die tatsächliche kontinuierliche
  Kameraframerate; die Konstante hat noch eine Vorprüfungsreferenz. Also nicht
  als „unbenutzt“ entfernen oder daraus 20 echte Kamerabilder/s ableiten.

Sinnvoll und beizubehalten: echte Quaternion-Prüfung, ToF-Transport-/Altersprüfung,
vormals korrigierte vollständige Logspeicherung, begrenzte Diagnostikbudgets,
Entprellung mit sofortiger LOW-Annahme, Geometrie-/Richtungsprüfung vor Bewegung,
mehrere echte Farbframes und geometrische Sitzzuordnung. Eine neue globale
Stoppregel oder pauschal strengere Farbschwelle folgt aus dieser Prüfung nicht.

Die aktuellen Modusschalter wählen regulär Obstacle mit drei Runden und finalem
Einparken. `OBSTACLE_PARKING_EXIT_TEST_ONLY=true` löst wegen gleichzeitig aktivem
Entry-Discovery **keinen** Exit-Endhalt aus; die Bedingung ist ausdrücklich
verknüpft. Kamera-Autostart betrifft nur den ausdrücklich gewählten stationären
Kalibriermodus. Diese beiden `true`-Werte sind hier kein versteckter Normalhalt.

## Umfang, Reproduktion und Grenzen

Geprüft wurden die lokalen Firmware- und Headerbereiche für Hauptschleife,
Freigabe/Moduswechsel, Motor/PID/Encoder, Gyro/ToF/RPC beider Kerne,
Kamera/GC2145/DMA, Vision/Blob-/Sitzzuordnung, Kartenaufbau,
Obstacle-Entdeckung/Connector/Pfad/Runde-2/3-Planung, Open-Navigation,
Park-/Ausparkzustände, Kalibrier-/Testmodusanbindung, serielle Befehle,
Logger und Telemetrie sowie die PlatformIO-Quellfilter. Die Bewegungs- und
Fehlerübergänge wurden vertieft geprüft; zusätzliche Suchläufe erfassten
blockierende Schleifen, Speicherzugriffe und Altcode. Das ist keine formale
Verifikation aller möglichen Zustände oder aller externen Bibliotheken.

Mit dem IDE-verwalteten Python, aus dem Repository:

```text
python simulation/review_runtime_edges_20261027.py
python simulation/check_later_laps.py
python simulation/check_later_lap_dynamics.py
python simulation/review_later_lap_matrix_20261027.py
```

Der letzte Befehl liefert bewusst Exitcode 1, solange die Reserveverletzungen
vorliegen. Er vergleicht beide Varianten über die gesamte Teilmatrix.
`review_runtime_edges_20261027.py` protokolliert Beobachtungen des aktuellen
Verhaltens; dessen Exitcode 0 ist keine Zusicherung sicheren Verhaltens.
Generierte C++-Dateien, Binärdateien und JSON-Ausgaben bleiben unter
`local_workspace/code_audit_20261027/` bzw. den vorhandenen Modellverzeichnissen.

Quellidentität der dynamischen Gegenprüfung, SHA-256 der tatsächlichen Dateien:

```text
src/obstacle_path.cpp 98392ce568a4f9f8d9b757b7256cb8640b3c61c43e67c3fc3ead8f9441aee5a8
src/motor_control.cpp d9fbb754a558dbc8bf969536885a059f10c2830e161fd148cc60b7e20016ca09
include/config.h     5f87069634849a6bb732d2762d6ece63df9d04f29d5c207a4b02565e10f93cb2
```

Keine neuen physischen Versuche, keine Bildserien, keine neue ToF-Abtastfahrt.
Reifenhaftung, Servoverzögerung, Versorgung und tatsächliche Montage bleiben
physisch unbestätigt. Bestehende Geometrieergebnisse sind Modellwerte, keine
gemessenen Berührungsreserven. Die früheren 1736 Erstlauf- und 396 Parkmodellfälle
wurden in der vorherigen Umsetzung geprüft; in dieser reinen Reviewrunde wurden
sie nicht pauschal noch einmal ausgeführt.

Reihenfolge der nächsten Implementierung: akzeptierte Aktuator-Wiederaufnahme
und Reglerzeitlücken, USB-/Fahrt-Koordination, sechs neue Innenlinienfälle plus
vorbestehenden Dynamikfall absichern, M4-I2C-Fehlerweitergabe, danach gezieltes
Aufräumen. Fehlende Original-Logs bei Verfügbarkeit zuerst unverändert sichern,
erst dann konkrete GREEN-/Einparkursachen mit der installierten Firmware
abgleichen. Keine Kamera-/Sitzschwellen aus Vermutung global verschärfen.
