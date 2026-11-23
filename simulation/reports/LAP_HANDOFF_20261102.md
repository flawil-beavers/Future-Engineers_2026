# Runde 1 -> 2: gezielte sanfte Übergabe

Der Nutzer hat den beobachteten Ruck auf diesen Rundenwechsel eingegrenzt.
Kein neuer Testlauf oder USB-Datensatz liegt für diese Änderung vor.

## Befund und Änderung

Die Steuerung wechselte gleichzeitig vom eingelesenen zum optimierten Weg und
von der geschwindigkeitsabhängigen Vorausschau auf die feste Vorausschau der
Folgerunden. Obwohl die ersten 200 mm der Wege identisch bleiben, kann das
Lenkziel bereits weiter vorne liegen. Die Analyse an identischer Modellpose
zeigt daher einen zusätzlichen Zielsprung beim Umschalten.

Die neue M7-Version `20261102_lap_handoff` speichert die zuletzt tatsächlich
ausgegebene Vorausschau und Sollgeschwindigkeit der Einleserunde. Nach erfolgreicher
Kartenprüfung und Routenberechnung werden über 400 mm gemessenen Vorwärtsweg
die beiden Lenkziele mit `w = 3t² - 2t³` überblendet. Die erste Ausgabe verwendet
noch das bisherige Ziel. Am Ende gilt vollständig das Ziel der optimierten Route.
Die Endsteigungen des Blendgewichts sind null. Die Ziele wandern mit dem Roboter;
es wird kein veralteter absoluter Zielpunkt festgehalten.

Auch die höhere Sollgeschwindigkeit wird mit diesem Gewicht freigegeben. Ein
niedrigeres neues Strecken- oder Kurvenlimit wird niemals angehoben. Die vorhandene
Beschleunigungsrampe und Sicherheitsprüfungen bleiben aktiv. Wartezeit verbraucht
keinen Übergangsweg, Rückwärtsbewegung setzt den Fortschritt nicht zurück. Die
Übergabe wird nur einmal beim Eintritt in Runde 2 aktiviert und beim Neustart
zurückgesetzt. Runde 2 -> 3, Parksteuerung, Kamera und Einlesen sind unverändert.

## Geprüfte Grenzen

`check_lap_handoff.py` extrahiert die Produktionsfunktionen. Für Aufstellung B
in CW und CCW mit 135, 180 und 260 mm/s aus Runde 1:

| Richtung | Geschwindigkeit | Zielsprung vorher | Zielsprung beim Umschalten jetzt |
|---|---:|---:|---:|
| CW | 135 mm/s | 136,339 mm | 0 mm |
| CW | 180 mm/s | 72,626 mm | 0 mm |
| CW | 260 mm/s | 12,674 mm | 0 mm |
| CCW | 135 mm/s | 132,170 mm | 0 mm |
| CCW | 180 mm/s | 68,229 mm | 0 mm |
| CCW | 260 mm/s | 6,424 mm | 0 mm |

Dies isoliert die Umschaltung bei gleicher Pose und gleichem Fortschrittsindex;
es behauptet keine vollständig stetige Servobewegung. Normale Wegpunktwechsel
und die bestehenden Kurven-Vorausschaugrenzen bleiben diskret. Der Test prüft
zusätzlich vollständige Zwei-Runden-Fahrten, Geschwindigkeitsgrenzen, Warten,
Rückwärtsrauschen, Ende der Überblendung, Rundenzähler und Reset.

`check_later_laps.py`: 2807/2807 bestanden mit aktiver Übergabe.
`check_final_parking_approach.py --approach-only`: 2970/2970 bestanden.
`check_later_lap_dynamics.py --full-matrix`: jeweils 2191/2191 bei 10, 20 und
50 ms Regeltakt bestanden (6573 insgesamt). Höchste modellierte Querbeschleunigung
0,683068 m/s² gegenüber dem Limit 0,8 m/s²; kleinster Pfostenabstand 31,7066 mm.
Zusätzlich jeweils 21/21 dynamische Fälle mit `--outgoing-speed 135` und
`--outgoing-speed 180` bestanden. Diese verwenden die vollständigen A/B/C-Wege
und die extrahierte Motor-Beschleunigungsrampe. Reifenschlupf, reale Servo- und
Sensorlatenzen sowie eine blockierende Routenberechnung sind nicht simuliert.

## Rechenzeit und Hardwaregrenze

Der Routenaufbau ist weiterhin synchron. Neu misst genau eine Logzeile
`[LAPS] Handoff planning_us=...` seine gesamte Dauer. Ohne aktuelle Hardwarelogs
ist nicht belegt, welchen Anteil eine Rechenpause am beobachteten Ruck hat.
Die Änderung entfernt den nachgewiesenen Umschaltsprung, garantiert aber noch
keinen ruckfreien realen Lauf. Es wurden keine zusätzlichen Pausen, Sensorabfragen,
Motoraktionen oder dichten Fahrlogs eingebaut.

M7 IDE-PlatformIO: SUCCESS; RAM 459224/523624, Flash 572368/786432.
Binary 575920 Bytes, SHA-256:
`99e6ecadb3d280c8d09895dca1e6d310a141d4c3624a58896002b6858f312fb5`.
M4 unverändert; kein Upload und keine Roboterbewegung.

Nächster Schritt: M7 normal hochladen, regulären CW-/CCW-Lauf speichern. Originale
zuerst sichern und hashen. Rechenzeit sowie `LATER_TRACK` unmittelbar vor/nach
Runde 1 -> 2 prüfen und mit der physischen Beobachtung vergleichen.
