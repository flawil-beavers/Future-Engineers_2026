# Auswertung der nachgelieferten Läufe 531–540

## Einordnung und Sicherung

Der Benutzer bestätigt: Dies sind die gestern beschriebenen Läufe **vor** der
neuesten M7/M4-Korrektur. Sie validieren oder widerlegen die neueste Firmware
nicht. Die zehn Originale wurden vor der Analyse unverändert kopiert und ihre
SHA-256-Werte mit dem Stick verglichen. Dateien, Bytezahlen und vollständige
Hashes stehen im [Beweisverzeichnis](evidence/parking_exit_diagnostics/README.md).
Die dort verwendete Datierung ist der Eingangstag; das Dateisystem des Sticks
meldet unbrauchbare Zeitstempel im Jahr 2098.

Alle zehn enthalten RUN_START und RUN_END, truncated=0; kein Logüberlauf.
531–534 sind CW, 535–540 CCW. Übersetzungseinheiten melden unterschiedliche
Buildzeiten; daraus lässt sich kein eindeutiger Firmware-Gesamthash ableiten.
Die Zuordnung der vier CW-Dateien zum früher beschriebenen Viererblock passt
zur Reihenfolge und zu den Ergebnissen. Einzelne Kontakte oder Eingriffe in
den CCW-Läufen sind ohne weitere Zuordnung nicht als autonomer Fehler bewiesen.

## Ergebnisse

| Log | Befund |
| --- | --- |
| 531 CW | Ausparken abgeschlossen; drei Runden bis zum Parkanlauf. Scan erreicht beide Schranken, verwirft Lücke 289,3 mm mit `dual_marker_geometry`. |
| 532 CW | S3, Station 2 bleibt unbekannt. Rückwärtsbeobachtung fordert −70 mm/s bei gesperrten Servoschreibzugriffen an, fährt nur etwa 2,2 mm und erreicht die Zeitgrenze. Danach verriegelt. |
| 533 CW | Ausparken und Fahrt bis zum Parkanlauf; vollständiger Scan, Lücke 285,6 mm verworfen. |
| 534 CW | Ausparken und Fahrt bis zum Parkanlauf; vollständiger Scan, Lücke 284,5 mm verworfen. Mehrere Gyro-Neustarts verlängern den Ablauf. |
| 535 CCW | Ausparken/Anschluss abgeschlossen, GREEN5 und GREEN7 sowie RED9 bestätigt. Danach Stillstand bei etwa (638, −541, 41°), Solltempo 252 mm/s. |
| 536 CCW | Ähnlicher Stillstand nach Anschluss bei etwa (637, −544, 42°), Solltempo 252 mm/s. |
| 537 CCW | Anschluss abgeschlossen; RED10 bestätigt, später Stillstand bei etwa (1231, 450, 64°), Solltempo 260 mm/s. |
| 538 CCW | Nach 2,356 s während der hinteren Positionierung manuell deaktiviert. Kein vollständiges Ausparken; keine autonome Abbruchursache geloggt. |
| 539 CCW | Anschluss abgeschlossen, später Stillstand bei etwa (860, −703, 28°), Solltempo 100 mm/s. Grosses unzugeordnetes grünes Bildgebiet am unteren Bildrand. |
| 540 CCW | Fährt bis zur Rückkehrseite, mehrere Farben bestätigt. Stillstand bei etwa (−1115, −490, 275°), Solltempo 256 mm/s. |

Bei 535/536/537/539/540 meldet der Motorregler nur 5,9–9,1 mm Fortschritt in
1000 ms. Das ist **kein angeforderter Wahrnehmungshalt**: Fahrt und Servoschreiben
sind freigegeben. Ein Kontakt, blockierender Gegenstand, falsche tatsächliche
Pose oder Antriebsproblem sind aus diesen Zahlen allein nicht unterscheidbar.
Vollständige Kamerabilder und unabhängige Fahrzeugpositionen fehlen. Deshalb
keine pauschale HSV-Änderung oder höhere Antriebskraft als Reaktion darauf.

532 belegt dagegen den im vorherigen Review reproduzierten Wiederanlauffehler.
Die aktuelle Version stellt die Servofreigabe an akzeptierten Rückwärts- und
Rückkehrbefehlen wieder her und friert die Phasenzeit während echter Gyrohalte
ein. Das wurde bereits gezielt korrigiert; diese alten Logs rechtfertigen keine
erneute Änderung der gleichen Bewegungsgeometrie. Gyro-Neustarts und ihre
etwa 1,6-s-Unterbrechungen bleiben reale Beobachtungen, deren Häufigkeit mit der
aktuellen Firmware erst durch neue normale Läufe bewertet werden kann.

## Korrektur an der Parkkantenberechnung

Der bisherige Code benutzt die letzte kurze Messung an der festen Schranke
und die erste kurze Messung an der beweglichen Schranke gleichzeitig als
**Länge des ToF-Kegels**. An den Übergängen mischen sich nahe Schranken- und
ferne Wandreflexionen. Beispielsweise enthält 533 am Ende der zweiten Schranke
171 mm gegenüber etwa 62–80 mm über der Schranke; die Wand liegt bei etwa
255–280 mm. Ein gültiger Messwert ist dort kein zuverlässiger Abstand zur Spitze.
Mit 11° Halbwinkel verbreitert eine Mischung von 170 statt 60 mm die beiden
abgeleiteten Kanten zusammen um ungefähr 42 mm. Das erklärt einen konkreten
Mechanismus für die beobachtete Überschätzung um 37–42 mm.

Die neue Funktion `markerEdgeFootprint()` schneidet beide Kegelrandstrahlen mit
der vorhandenen Schrankenebene bei Y = Aussenwand + Parkplatzbreite. Sie nutzt
die reale Sensorposition und Fahrzeugrichtung. Die Messwerte bestimmen weiterhin
die bestätigte Wand/Schrankenfolge; nur die geometrische Kantenprojektion benutzt
keinen Mischabstand mehr. Nicht endliche Posen und rückwärts gerichtete Strahlen
liefern ungültige Kanten. Zwei kurze EDGE-Einträge und
`geometry=tip_plane_v1` machen den neuen Rechenweg in normalen Logs sichtbar.

Die Lückentoleranz bleibt ±15 mm, die maximale Positionskorrektur bleibt
unverändert, ebenso Bremsen, Capture, sieben Parksegmente, Geschwindigkeiten
und Kollisionsprüfungen. Der alte Radialhelfer wurde aus der Firmware entfernt
und für den unabhängigen bisherigen Sensorsimulator im Host-Test erhalten.

**Grenze des Nachweises:** Die alten Parktraces sind nur etwa alle 200 ms
gespeichert, nicht an jedem Kantenwechsel. Die exakten damals verwendeten
Kantenmessungen sind damit nicht vollständig rekonstruierbar. Dies ist ein
reproduzierter Quellcodefehler mit passendem Logmuster, kein exakter Replay-Beweis,
dass diese Korrektur alle drei realen Einparkversuche erfolgreich beendet hätte.
Reale Kegelform, Reflexionen, Schrankenposition und Schlupf bleiben zu prüfen.

## Prüfungen und nächste Schritte

- `check_parking_mixed_edge_ranges.py`: 864 Fälle mit unabhängig berechneter
  Schrankenebene, CW/CCW, ±3° Richtung, unterschiedlichen Scanabständen,
  ±180 mm Positionsversatz und 0/150/170/180-mm-Randmischungen bestanden.
  Geometrischer Lückenfehler unter 2 mm; echte 280-mm-Lücken werden verworfen.
  Der eingefrorene alte Rechenweg verwirft in 162 dieser passenden
  170-mm-Mischfälle eine gültige Lücke. Zusätzlich ungültige Pose/Richtung geprüft.
- `check_final_parking_robustness.py`: 1540 Szenarien, kein unsicherer Modellfall;
  alle 990 bekannten Anfahrten eingeparkt. Dazu 46 Rand- und 5 Qualitätsprüfungen.
- `check_final_parking_sequence.py`: 396 vollständige Folgen und 5
  Registrierungsprüfungen bestanden, einschliesslich Sensorhaltefällen.
- `check_parking_segment_sensor_resume.py`: 42 Prüfungen und negative Kontrolle
  bestanden. Diese Modelle enthalten keine gemessene Hardwareverzögerung.
- Auspark- und ToF-Sichtfächeranalyse aller zehn Originale unter
  `local_workspace/batch_20261027_531_540/pose/`. Neun Ausparkfolgen vollständig
  und ungekürzt; 538 bleibt als abgebrochene Folge enthalten. Hintere
  Encoder/ToF-Abweichung Median +1,51 mm, maximal 9,35 mm, alle neun innerhalb
  der bestehenden Unsicherheit. Kein Anlass für Backlash- oder Neutral-Korrektur.

Nur M7 ist durch diese zusätzliche Korrektur betroffen. Die bereits erforderliche
M4-Aktualisierung aus der vorigen Version bleibt erforderlich, falls noch nicht
aufgespielt. Kein Upload oder Fahrbefehl durch den Agenten. Anschliessend normale
CW/CCW-Läufe mit normaler Speicherung; zuerst wieder Originale sichern, danach
Kantenwerte, tatsächliche Parklage und Verhalten an den früheren Stillstandsorten
bewerten. Keine zusätzliche ToF-Fahrt oder Messpause nötig.

Finaler IDE-M7-Build SUCCESS: RAM 442408/523624, Flash 524712/786432.
SHA-256 des M7-Binaries:
`d3f65f7fad458849edd2c0b192b6b265b7b5d33ee868695945b81a1d00a32c30`.
M4 unver?ndert, nicht erneut gebaut; vorhandener aktualisierter M4-Hash:
`fbc3da65845172d9be1405ec56b626da1b716999ce2fd31e591a4c3a14057215`.
Runtime-Fehlerfallpr?fung erneut bestanden: alle vier Recovery-Freigaben,
Gyrohalte-Timer, Profilneustart, USB-Koordination und I2C-Fehlerpropagierung.
