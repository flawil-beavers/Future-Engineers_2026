# Open Challenge: Code- und Hostprüfung vom 27.10.2026

Erneut geprüft am 28.10.2026: siehe den Nachtrag am Ende. Der dortige Vergleich
bestätigt unveränderte Wandregelung, aber die zeitliche Regression der ersten
Eckbestätigung und eine bereits ältere Verzögerung im Distanzfilter.

Stand: `cc0bc59232359435018cc12281b488242767f96a`, zu Beginn sauberer
Arbeitsbaum. Keine Firmwareänderung, kein Upload, keine neue Roboterfahrt.
Die Open Challenge ist weiterhin vollständig enthalten und anwählbar.
Eine unveränderte praktische Funktion kann wegen der folgenden Befunde nicht
bestätigt werden.

## 1. Hohe Priorität: neue Verzögerung vor der ersten Kurve

In [navigation_controller.cpp](../src/navigation_controller.cpp) fordert
`state_following()` bei unbekannter Seite weiterhin **300000 µs** ToF-Budget an.
Seit `cc0bc59` verlangt die Eckbestätigung drei unabhängige Sequenzen derselben
Seite; vorher wurden drei Schleifendurchläufe derselben Messung gezählt.

Die unabhängigen Messungen beseitigen einen realen Bestätigungsfehler, ändern
aber den Zeitpunkt des Abbiegens erheblich. Mit im Host eingespeisten
300-ms-Messabständen erfolgt `NAV_TURNING` in CW und CCW **600 ms nach der
ersten positiven Messung**. Bei konstant 300 mm/s sind das weitere **180 mm**.
Die Phasenlage bis zur ersten Messung und die reale Mess-/Transportlatenz
kommen hinzu. Das ist eine Zeit-/Wegabschätzung, keine am Sensor gemessene
Latenz oder nachgewiesene Kollision. Der aktuelle Startstandard ist 300 mm/s.

Als eingefrorene Referenz reproduziert `861ed8e` dieselbe Bestätigung nach zwei
weiteren 1-ms-Schleifen ohne neue Messung. Die Änderung betrifft Open direkt;
sie ist keine Nebenwirkung der späteren Obstacle-Routenoptimierung. Nach der
Seitenauswahl wird das normale 30-ms-Budget wiederhergestellt; dort beträgt
die entsprechende Wartezeit bei 30-ms-Frames ungefähr 60 ms nach dem ersten
positiven Frame.

Nächster Schritt: Messbudget, unabhängige Bestätigung und verfügbare räumliche
Kurvenreserve gemeinsam auslegen. Nicht einfach zur Mehrfachzählung eines
alten Frames zurückkehren. Den tatsächlichen ersten Abbiegepunkt anschließend
in normalen CW-/CCW-Fahrten prüfen.

## 2. Hohe Priorität: Rundenende hängt von der Wandmessung ab

`state_following()` setzt `NAV_STOPPED` nach zwölf Kurven und mehr als 500 mm
auf der neuen Geraden ausschließlich innerhalb von
`0 < current_wall_distance < nav_wall_margin`. Die Eckprüfung steht davor.

Reproduktion in beiden Richtungen: zwölf Kurven, drei gezählte Runden,
501 mm seit Kurvenende, drei frische Gap-Frames. Ergebnis: **13. Kurve** statt
Stopp. Bei vollständig fehlenden/veralteten Seitenframes fährt der Code nach
1000 mm weiterhin mit dem normalen Geschwindigkeitsziel. Das ist ein
reproduzierter Zustandsfehler; eine physische Überfahrt wird damit nicht
behauptet. Der Gap-Fall ist bereits in `861ed8e` vorhanden.

Nächster Schritt: Abschlussdistanz/Rundenziel von der Verfügbarkeit einer
Wandkorrektur trennen und verhindern, dass nach erreichtem Abschlussziel eine
weitere Ecke Vorrang bekommt. Normalen Softstopp und einmaliges Speichern
beibehalten; frische Gaps, ungültige und veraltete Frames getrennt prüfen.

## Weitere reproduzierte Risiken

| Befund | Nachweis und Bedeutung | Nächster Schritt |
| --- | --- | --- |
| Open übernimmt vorherige Navigationsgeschwindigkeit | Obstacle-Setup setzt `nav_normal_speed` auf 220; der tatsächliche `MODE_OPEN_CHALLENGE`-Start setzt nur Obstacle-Modus aus und ruft `navigation_enable()` auf. Im Host bleibt das Ziel 220 statt des Kaltstartwerts 300 mm/s. | Modusspezifische Standardwerte bzw. ausdrücklich gemeinsame Benutzereinstellungen definieren. |
| Keine Zeit-/Weggrenze für die Kurve | `state_turning()` beendet nur bei Überschreiten des Gyro-Zielwinkels. Mit angenommen gesundem, aber unverändertem Winkel und fortschreitendem Encoder bleibt es nach simulierten 10 s/3000 mm in `NAV_TURNING`. Der Stillstands-Watchdog deckt Fahrt mit Encoderfortschritt nicht ab. | Plausibilitäts-/Abbruchgrenze für eine nicht fertig werdende Kurve definieren; frische Gyrodaten allein beweisen keine Zielerreichung. |
| Langsames Messbudget bleibt nach Abbruch bestehen | `navigation_disable()` während unbekannter Seite lässt im Host 300000 µs und `nav_long_range_active` bestehen. Es stellt weder Budget noch Suchflag zurück. | Sensorzustand beim Modusende und beim Start des folgenden Modus eindeutig wiederherstellen. |

Die ersten beiden Tabellenbefunde und die fehlende Budgetbereinigung sind
Quellcode-/Hostbefunde; kein neuer physischer Ausfall wird ihnen zugeordnet.
Die Kurvenprüfung setzt absichtlich einen unveränderten, als gesund behandelten
Winkel voraus. Ob ein solcher Fehler real auftritt, ist nicht bewiesen.

## Was weiterhin funktioniert

Der vollständige aktuelle Navigationscode wurde mit tatsächlichen
`stop`, `set_speed`, `set_steering`, `steer` und den aktuellen
ToF-Frische-/Snapshot-Zugriffen ausgeführt. Simuliert sind Sensorframes,
Bewegung, Winkel, elektrische Ausgabe und USB-Speicherung.

- CW/CCW: zwölf Kurven, korrekte Gyro-Zielwinkel, normaler Abschluss bei
  vorhandener Wand, kontrolliertes Abbremsen und genau ein Speicherauftrag.
- Mehrere Schleifen derselben ToF-Sequenz bestätigen keine Ecke; veraltete
  oder transportfehlerhafte Messungen löschen die Bestätigung. Nach frischen
  Frames erholt sich die Erkennung.
- Die bestätigte Seite bestimmt die erste Drehrichtung; das Messbudget wird
  nach der Seitenauswahl zurückgestellt.
- Beide bewegten Open-Zustände stellen nach `stop(false)` Motor und Servo
  wieder her. Die jüngste Servo-Korrektur bleibt wirksam.
- Die Open-Auswahl startet keine Kamera-/Hindernisroutenoptimierung.
  `STARTUP_ROBOT_MODE` ist derzeit weiterhin `MODE_OBSTACLE_CHALLENGE`.
  Seriell wählt das kleine `l` Open bis zum nächsten Reset; ein autonomer
  Open-Kaltstart erfordert die entsprechende M7-Konfiguration.

## Reproduktion und Build

Aus dem Repository-Stamm, Python mit Host-`g++` oder `clang++` auf PATH:

```powershell
$py = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/python.exe'
& $py simulation/review_open_challenge_20261027.py
& $py simulation/check_runtime_safety.py
& $py simulation/check_drive_profile_timing.py
& $py simulation/review_runtime_edges_20261027.py
```

Alle vier Aufrufe erfolgreich. Das neue Werkzeug enthält ausdrücklich
`FINDING`-Reproduktionen; Exitcode 0 bedeutet, dass die Erwartungen des Audits
reproduziert wurden, nicht dass Open fehlerfrei abgenommen ist. Generierte
Dateien bleiben unter `local_workspace/open_challenge_review_20261027/`.
Die bestehende Profilprüfung bestätigt 40000 normale Rampenvergleiche gegen
die historische Implementierung mit maximaler Geschwindigkeitsdifferenz 0.
Die Laufzeittests bestätigen unter anderem Gyro-Erholung, Frische,
USB-Startkoordination und die zuvor korrigierten Zeitlücken.

IDE-verwalteter inkrementeller Build ausschließlich `giga_r1_m7`: **SUCCESS**.
RAM 442408/523624 Bytes, Flash 524296/786432 Bytes.
SHA-256 `.pio/build/giga_r1_m7/firmware.bin`:

```text
5542896c58ca0c36b2c389c0be2d63471b2a0dbeeb27a1cbedee4b44ab7b22d8
```

M4 unverändert und nicht zusätzlich gebaut. Kein Clean, kein separater Cache,
kein Upload. Der Build prüft die vorhandene Firmware mit anwählbarem Open;
die Startup-Auswahl wurde dafür nicht geändert. Physische Kurvenreserve,
Sensorlatenz und Streckenabschluss bleiben durch diese Hostprüfung unvermessen.

## Nachprüfung 28.10.2026: einfache Innenbanden-Fahrt und jüngste Änderungen

Geprüft wurde `16db81b8bc55833ef755009f4ea0d6d51ce9c671` samt den bereits
vorhandenen Einparkänderungen im Arbeitsbaum. Diese Sitzung verändert nur
Prüfwerkzeuge und Dokumentation. Die vorige Open-Prüfung hatte ebenfalls keine
Firmware verändert; die fragliche Eckbestätigung stammt aus der früheren
Laufzeitkorrektur `cc0bc59`.

**Innenbanden-Regelung unverändert:** Vergleich gegen `861ed8e` in 560 Szenarien
mit 11200 einzelnen ausgegebenen Befehlen. Linke/rechte Bande, Abstand
200/250/300/350/550 mm, sieben Winkelabweichungen von -12 bis +12 Grad,
Schleifenabstände 1/10/30/50 ms und aktive Wand-Wiedererfassung ein/aus.
Lenkbefehl, simulierte Servoausgabe, Geschwindigkeitsziel und Wiedererfassung
sind in jedem verglichenen Schritt gleich. Die Korrekturrichtung stimmt:
bei zu kleinem Abstand weg von der jeweiligen Bande, bei zu großem Abstand
zur Bande hin. Der 300-mm-Sollabstand, PD-Verstärkungen, Gyrofilter und
Kurvenbefehl +/-25 bleiben gleich. Open ruft weder Kamerabildauswertung noch
den Obstacle-Pfadplaner auf.

Die zuletzt vorbereiteten Firmwareänderungen liegen in `final_parking.cpp`
und einer Diagnose-Revisionszeichenfolge in `obstacle_path.cpp`. Die
Einpark-Zustandsmaschine wird beim normalen Open-Ablauf nicht ausgeführt.
Gemeinsame Motor-/Gyroänderungen seit dem ersten Review wurden trotzdem erneut
geprüft: Laufzeitsuite und 40000 normale Profilvergleiche bestehen. Auch die
neuere Behandlung wiederholt langsamer Regelschleifen besteht ihre Prüfung.
Das beweist keine unveränderte reale Sensorlatenz.

**Bestätigte Regression der ersten Ecke:** Der neue Zusatztest führt jetzt
auch `read_single_tof()` aus, statt nur bereits gefilterte Distanzen
einzuspeisen. Die Hardware liefert im Test alle 300 ms einen Frame; zwischen
den Frames werden schnelle Navigationsschritte ausgeführt.

| Eingespeister Übergang an der inneren Seite | Referenz 861ed8e | Aktueller Stand |
| --- | --- | --- |
| 300 mm Wand, dann kein erkanntes Objekt | Ecke beim ersten neuen Frame | Ecke beim dritten neuen Frame |
| 300 mm Wand, dann Hardware-Objekt mit zu schwachem Signal | Ecke beim ersten neuen Frame | Ecke beim dritten neuen Frame |
| 300 mm Wand, dann gültiger, akzeptierter 2000-mm-Abstand | Ecke beim 13. neuen Frame | Ecke beim 15. neuen Frame |

Beide Richtungen liefern dieselben Ergebnisse. Die erste Bestätigung braucht
damit nach der ersten Gap-Messung etwa 600 ms zusätzlich statt weiterer
Schleifenticks. Bei 300 mm/s entspricht dies etwa 180 mm Fahrt. Unabhängige
Frames sind als Entstörung sinnvoll, aber das bestehende 300-ms-Suchbudget wurde
bei dieser Änderung nicht passend berücksichtigt. Dies ist ein echter
zeitlicher Rückschritt gegenüber dem alten Trigger, kein nachgewiesener
physischer Unfall.

**Ältere Filter-Schwäche zusätzlich nachgewiesen:** Ein gültiger weiter
Wandabstand wird im Suchmodus pro Frame auf höchstens +100 mm begrenzt.
Von 300 mm aus überschreitet er deshalb erst beim 13. Frame die strikte
1500-mm-Eckschwelle. Aktuell folgen zwei weitere Bestätigungen. Der synthetische
Fall benötigt rund 4.2 s nach dem ersten weiten Frame, früher rund 3.6 s.
Das tritt nur unter den eingespeisten Bedingungen dauerhaft gültiger weiter
Rückläufer auf; echte Leer-/Out-of-range-Frames umgehen den Filter. Ob reale
schwarze Wände solche Rückläufer liefern, wurde hier nicht gemessen. Das
Filterproblem darf nicht vollständig der neuen Bestätigung zugerechnet werden.
Ein frisch transportierter, aber qualitativ verworfener Rückläufer wird im
bestehenden Sensorpfad ebenfalls als Gap gewertet; drei neue Frames allein
beweisen deshalb noch kein geometrisches Wandende.

Der bereits dokumentierte wandabhängige Rundenabschluss wurde erneut
reproduziert. Die bisherigen Drei-Runden-Tests injizieren passende Winkel und
Distanzen; sie sind Zustandsprüfungen und keine geschlossene physische
Simulation der kompletten Innenbanden-Fahrt.

Reproduktion zusätzlich zu den Befehlen oben:

```powershell
& $py simulation/check_open_wall_following_20261028.py
```

Arbeitsdateien: `local_workspace/open_wall_following_20261028/`. Der Test
erzeugt die Basisfixtures selbst. `FINDING`-Ausgaben bleiben ungelöste Befunde.
IDE-M7-Build erfolgreich, RAM 442416/523624, Flash 532888/786432 Bytes;
SHA-256 `ca500e0d2b6127929fa0ac99cf98164cd8e510be535fc7c708b3b12425a0f4d9`.
M4 unverändert, kein zusätzlicher M4-Build und kein Upload.

Nächste gezielte Arbeit: erste Eckbestätigung samt tatsächlicher Sensorperiode
und Behandlung großer gültiger Distanzsprünge abstimmen; Rundenabschluss von
der Wandkorrektur entkoppeln. Die nachgewiesen unveränderte einfache
Innenbanden-Regelung braucht dafür keinen neuen Routenplaner. Eine globale
Änderung der geteilten ToF-Filter würde zusätzlich Obstacle beeinflussen und
sollte ohne passende Nachweise vermieden werden.
