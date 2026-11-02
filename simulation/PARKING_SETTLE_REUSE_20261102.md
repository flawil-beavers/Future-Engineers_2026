# Kürzere Parkpausen mit Stillstandsnachweis

Parkversion `20261102_reuse_steer_settle`, Pfad `20261102_lap_handoff`.
Der Nutzer hat nach der Zeitauswertung ausdrücklich die Verkürzung der Pausen
beauftragt. Keine neue Hardwarefahrt oder Loglieferung in dieser Sitzung.

## Änderung

1. Die Motorsteuerung merkt Zeitpunkt und Winkel des tatsächlich ausgeführten
   Servo-Schreibbefehls. Eine bloss vorgemerkte oder wegen gesperrter Lenkung
   unterdrückte Anforderung zählt nicht. Die Parksteuerung kann bereits
   vergangene Einstellzeit bei **identischem** Winkel wiederverwenden. Neue
   Winkel erhalten weiterhin volle 400 ms. Der Servo hat keine Positionsrückmeldung;
   dies ist eine Zeitfreigabe, kein gemessener Radwinkel. `stop()` sperrt nur neue
   Schreibbefehle, die Servo-PWM bleibt aktiv.
2. Der nächste geprüfte Lenkwinkel wird direkt beim Eintritt in die Einstellphase
   geschrieben. Seine Wartezeit beginnt damit beim wirklichen Schreibbefehl und
   nicht eine Regelschleife davor oder danach. Die stationäre Kollisionsprüfung
   bleibt davor und während der Einstellphase aktiv.
3. Nur beim Einparken, an den Haltepunkten nach Anfahrt/Segment/Neuplanung, darf
   die Bremsphase früher enden. Dafür müssen mindestens 150 ms seit Bremsbeginn
   und 120 ms beobachtete Ruhe vergangen sein: Encoderänderung höchstens 0,5 mm,
   Winkeländerung höchstens 0,25 Grad, mindestens zwei neue Geschwindigkeits-
   messungen und aktuell höchstens 5 mm/s. Bewegung setzt das Ruhefenster zurück.
   Sensorpausen und Reset löschen diesen Nachweis. Ohne frühen Nachweis gilt
   unverändert der bisherige 300-ms-Pfad mit Geschwindigkeitsprüfung und Timeout.

Routen, Parkgeometrie, Geschwindigkeiten 60/40 mm/s, Suchalgorithmus, Reserven,
Zielposition und Ausparkdiagnostik wurden nicht verändert. Die übrigen Brems-
phasen der Parkanfahrt/Vermessung bleiben beim bisherigen Verhalten.

## Prüfung

`check_parking_steering_wait.py` extrahiert die realen Servo-Funktionen und
Parksteuerung. Geprüft: unbekannter Winkel, unterdrückter Schreibbefehl,
vorgemerkter versus ausgeführter Winkel, 399/400-ms-Grenze, wiederholter Befehl,
Timerüberlauf, CW/CCW-Segmentbeginn und Abschluss, Bremsmindestzeit, fehlende
neue Messungen, Encoderbewegung, Winkelbewegung, zu hohe/ungültige Geschwindigkeit
und Sensorpause. Alle bestanden.

Weitere bestanden: 396 vollständige Parkabläufe + fünf Registrierungsfälle,
84 Bremsnachlaufvarianten bis 3 mm + acht Gesundheitssperren, 14 Platz-/Fortschritts-
prüfungen und 36 Abläufe mit Radius-/Winkelabweichungen. Zeitmatrix: 216/216 ohne
Modellkollision, nochmals 216/216 mit realer Encoder-Geschwindigkeitsfilterung
(50-ms-Mindestfenster, Alpha 0,60). Keine Hardwaremessung der neuen Version.

Vergleich ab registrierter Halteposition, mit Beschleunigungsrampe und
Geschwindigkeitsfilter. `--legacy-waits` stellt nur die alten Wartebedingungen
im Hostmodell wieder her; gleiche Geometrie, gleiche Startlagen, gleiche
Sensoradapter. Beide Richtungen ergeben bei gespiegelten Eingaben dieselben Zeiten:

| Lücke / Beispiel | Vorher | Jetzt | Ersparnis |
|---|---:|---:|---:|
| 232,5 mm | 15,08 s | 13,84 s | 1,24 s |
| 245,2 mm / aufgezeichnete Startpose | 15,34 s | 13,90 s | 1,44 s |
| 247,5 mm | 12,24 s | 11,42 s | 0,82 s |
| 251,7 mm | 12,20 s | 11,42 s | 0,78 s |
| 256,6 mm / aufgezeichnete Startpose | 12,20 s | 11,48 s | 0,72 s |
| 262,5 mm | 12,20 s | 11,42 s | 0,78 s |

Die neue gefilterte 216er-Matrix reicht von 10,40 bis 19,28 s. Das ist kein
garantierter Höchstwert. MCU-Suchzeit, vorherige Anfahrt/Vermessung, USB-Speicherung,
unbekannte Posefehler, Schlupf und echte Aktuatorabweichungen sind nicht enthalten.
Die allgemeine geometrische Neuplanung kann weiterhin längere Rechenpausen
verursachen; diese Änderung beschleunigt die Freigaben zwischen den Bewegungen.

Reproduktion: `audit_parking_duration.py --filtered-speed`; Vergleich
`audit_parking_duration.py --nominal --filtered-speed --legacy-waits`.
Ausgaben unter `local_workspace/parking-duration-filtered/` beziehungsweise
`local_workspace/parking-duration-nominal-filtered-legacy/`.

## Build und nächste Prüfung

M7 IDE-PlatformIO SUCCESS, RAM 459256/523624, Flash 572912/786432.
Binary 576464 Bytes; SHA-256:
`03fadef7953736ca4b8fb3e220300dfadc5e6c60457c31344f3713cb78870d2b`.
M4 unverändert, kein Upload oder Fahrbefehl.

Nächste normale CW-/CCW-Läufe mit dieser M7-Version speichern. Originale zuerst
sichern/hashen und Abziehen bestätigen. Tatsächliche Segment-/Bremszeiten,
`search_ms`, Bremsnachlauf und Kontakte/Reserven vergleichen. Keine zusätzlichen
ToF-Testbewegungen erforderlich.
