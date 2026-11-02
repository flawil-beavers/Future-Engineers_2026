# Parkdauer und Platzbedarf erneut geprüft

Geprüfte Firmware: Pfad `20261102_lap_handoff`, Parken
`20261101_efficient_entry`. Keine neue Hardwarefahrt/USB-Evidenz erhalten.
Die Firmware wurde in dieser Prüfung nicht verändert oder hochgeladen.

## Ergebnis

Der aktuelle Ablauf kann in den geprüften CW-/CCW-Modellen vollständig und
kollisionsfrei einparken. Eine stets sehr schnelle Ausführung ist damit nicht
belegt. Insbesondere sind Anzahl der ursprünglich geplanten Bögen, tatsächlich
ausgeführte Korrekturen und die gesamte Parkzeit unterschiedliche Grössen.

Die folgenden Zeiten beginnen beim Stillstand an der bereits registrierten
Position vor dem Parkfeld (`FP_CAPTURE_BRAKE`) und enden beim bestätigten
Parkabschluss. Sie enthalten die aktuelle Beschleunigungsrampe, Brems- und
Lenkwartezeiten, aber **keine MCU-Rechenzeit, USB-Speicherung oder vorherige
Anfahrt/Vermessung**. Es sind Modellzeiten, keine gemessenen Roboterzeiten.

| Gemessene Lücke | Nominelle Modellzeit | Gefahrene Teilstücke / Bögen | Zeitspanne der Varianten |
|---|---:|---:|---:|
| 232,5 mm | 15,08 s | 6 / 3 | 14,04–21,44 s |
| 245,2 mm, aufgezeichnete Startpose | 15,34 s | 7 / 3 | 12,48–16,54 s |
| 247,5 mm | 12,24 s | 6 / 4 | 12,12–16,40 s |
| 251,7 mm | 12,20 s | 6 / 4 | 12,06–15,98 s |
| 256,6 mm, aufgezeichnete Startpose | 12,20 s | 6 / 4 | 12,00–16,28 s |
| 262,5 mm | 12,20 s | 6 / 4 | 10,94–15,98 s |

CW und CCW sind bei gespiegelten Eingaben gleich. Varianten: tatsächlicher
Radiusfaktor 1 / 1,136 / 1,235, anfänglicher Winkelversatz -1,5 / 0 / +1,5 Grad,
zusätzlicher Bremsnachlauf 0 / 1,5 mm, in beiden Richtungen: 216 Fälle.
Der Planer beginnt mit Radiusfaktor 1 und lernt den Radius aus der Bewegung.
Der Versatz ist in der geschätzten Pose sichtbar; unbekannte Gyrofehler oder
Reifenschlupf sind damit nicht abgedeckt. 21,44 s ist kein garantierter Höchstwert.

## Zeitaufteilung

Beim nominellen Beispiel mit 251,7 mm Lücke:

- etwa 333,8 mm Fahrt, reine Strecke/60 mm/s: 5,56 s;
- mit Anfahren, kurzen Teilstücken und Regeltakt: 7,26 s in Fahrzuständen;
- sechs Lenk-Einstellphasen: 2,40 s;
- Bremsphasen einschliesslich Start: 2,12 s;
- Abschlussprüfung/Lenkung neutral: 0,42 s;
- zusammen 12,20 s, zuzüglich echter Berechnungsdauer.

Normale Einfahrgeschwindigkeit ist 60 mm/s. Die enge Ausweichplanung fährt
40 mm/s. Jede echte neue Lenkeinstellung wartet 400 ms; jeder Segmentabschluss
prüft mindestens 300 ms Bremsruhe und zusätzlich die gemessene Geschwindigkeit.
Diese physikalischen Zeiten wurden mangels neuer Aktuatormessung nicht gekürzt.

Am Beispiel 245,2 mm liefert der reine Planungstest fünf Teilstücke mit zwei
Bögen und einem Richtungswechsel. Das allein belegt **nicht**, dass der neue
Zweibogen-Sonderplan mit gewöhnlicher Reserve gewählt wurde: Der vollständige
Ablauf wählt hier die schnelle Rezeptsuche mit 1-mm-Modellreserve und 40 mm/s.
Seine laufende Vorausschau fordert eine `entry_motion_clearance`-Korrektur vor
dem zweiten Bogen. Daraus werden sieben ausgeführte Teilstücke, drei Bogenstarts
und 15,34 s statt einer aus fünf geplanten Segmenten abgeleiteten Idealzeit.
Die frühere Beschreibung der 5/2/1-Zahlen darf nicht als gemessener schneller
Parkabschluss oder als Nachweis gewöhnlicher Reserve gelesen werden.

## Geometrie

Konfiguriert sind 123 mm Gesamtbreite, 30 mm Heck ab Hinterachse und 125 mm
Frontausdehnung. Der Kollisionsprüfer berücksichtigt zusätzlich die einzelnen
eingeschlagenen Räder, beide rosa Begrenzungen, Aussenwand und den bevorzugten
16-mm-Frontfreiraum beim Lenken im normalen Reservemodus. Der zusätzliche frühere
10-mm-Heckwunsch bleibt aufgehoben. Das bekannte Endziel bleibt lokal 60/127 mm.

Zur Grössenordnung: Bereits die rechteckige Grundkontur benötigt bei Winkel a
längs der Lücke `155*cos(a) + 123*abs(sin(a))` mm. Bei 45 Grad sind das rund
196,6 mm. In einer 247,5-mm-Lücke bleiben somit nur rund 50,9 mm insgesamt,
**vor** Räderschwenk, Reserve und Bremsweg. Deshalb kann man nicht bei jedem
Rangierzug pauschal 60 mm freigeben. Die Firmware prüft die gesamte Bewegung
millimeterweise und berechnet freie Vor-/Rückfahrt aus der jeweiligen Pose.

## Verifikation und verbleibende Optimierung

- `audit_parking_duration.py`: 216/216 vollständig eingeparkt, keine geprüfte
  physische Konturüberschneidung; echte `DriveProfile`-Rampe bei 20 ms.
- `check_final_parking_sequence.py`: 396/396 eingeparkt, fünf Registrierungschecks.
- `check_parking_efficient_entry.py`: 14 Platz-/Fortschrittsprüfungen und
  36/36 vollständige Abläufe mit Radius-/Winkel-/Bremsabweichungen.
- Testadapter liest jetzt den aktuellen Produktionsstand direkt und verwendet
  nur einen Encoderadapter; vermeidet veraltete extrahierte Routenfixtures.

Die Kostenfunktion gewichtet gegenwärtig Weg, Segmentanzahl und Richtungswechsel;
sie berechnet keine tatsächliche Ausführungsdauer. Die zusätzlichen 10 mm pro
Segment entsprechen bei 60 mm/s nur 0,167 s gegenüber ungefähr 0,7 s Bremsen
plus Lenken. Das ist ein Ansatz für eine gezielte spätere Zeitoptimierung,
kein Beweis, dass eine andere gefundene Route sicher schneller wäre.

Auch die allgemeine Ausweichsuche hat ein Geometrieschrittbudget, aber keine
garantierte MCU-Zeitgrenze. Alte Logs 633/634 stammen vor der aktuellen
Parkversion; dortige Suchpausen bis etwa zehn Sekunden dürfen nicht als aktuelle
Messwerte ausgegeben werden. Der nächste normale Lauf muss `search_ms`, wirklich
ausgeführte Segmente, Reserven und Parkdauer bestätigen. Keine neuen Fahrbefehle,
pauschalen Abstandsverkleinerungen oder Fahrgeschwindigkeitserhöhungen vorgenommen.

Reproduzierbare Ausgaben: `local_workspace/parking-duration/report.json` und
`output.txt`; die nominale Detailausgabe kann mit `--nominal` erzeugt werden.
