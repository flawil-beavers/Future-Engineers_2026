# Nur das Rückwärts-Einparken üben

Die aktuelle Version startet nach Reset wieder im vollständigen
`OBSTACLE_CHALLENGE`. Für das separate Training bei ausgeschalteter Fahrfreigabe
`park cw` oder `park ccw` auswählen. Im Parktraining: keine Ausparkbewegung, keine Runden und keine Anfahrt
in die entfernte Ecke. Sie beginnt am Übergabepunkt direkt vor dem
Rückwärts-Einparkmanöver und benutzt danach denselben Einparkplaner wie der
normale Hindernislauf, einschliesslich Endkorrektur und Halten nach Abschluss.
Die Kamera wird für diesen isolierten Modus nicht initialisiert.

Stand nach616 und Masskorrektur: Gerade Gesamtbreite inklusive Raeder123mm.
Neu gemessen: Hinterachsmitte bis hinterster Teil inklusive Sensor30mm.
Einpark-Koerperflaechen, Endlagen- und Reservepruefung verwenden jetzt30mm
statt40mm. Die Frontkontur und die Hinterachs-Zielposition60mm bleiben gleich.
Die gute Endposition in Laengsrichtung bleibt erhalten (Ziel Achse60mm,
nominal30mm hinten/62,5mm vorne bei24,75cm Parkluecke); die irrtuemliche
5mm-Verschiebung und die neue zusaetzliche Endpositionssperre sind aufgehoben.
Die bevorzugte zusaetzliche Reserve VORNE beim Lenken ist jetzt16mm
(nochmals8mm hinzugefuegt). Die normale Planung prueft zuerst einen16mm-
Rueckwaertspraefix, um den vorhandenen Platz hinten zu nutzen. Jede Bewegung
braucht die vollstaendige Kollisionspruefung. Die enge Rettungsplanung behaelt
ihren bisherigen8mm-Praefix und kann die bevorzugte Zusatzreserve unterschreiten;
die reale Rad-/Aufbaugeometrie wird dabei weiterhin geprueft.
Nach Log628 ist die bevorzugte zusaetzliche10mm-Rueckwaertsfolge entfernt:
Die korrigierten30mm Heckueberhang ersetzen diesen frueheren Wunsch. Die normale
Planung prueft vorwaerts und rueckwaerts samt vollstaendiger Fortsetzung und
nutzt den geometrisch verfuegbaren Raum. Kein pauschaler Rueckwaertszuschlag
wird bei Neuplanungen wiederholt. Die gute Endposition bleibt unveraendert.
Bevorzugte seitliche Achsposition127mm ab Bande statt125mm: nur2mm flacher.
Bei123mm Breite bleiben nominal11,5mm zur offenen Parkkante, vor Messfehlern.
Die flachere Folge wird zuerst geprueft; sichere tiefere Alternativen bleiben,
falls die flachere Anfahrt nicht vollstaendig kollisionsfrei passt.
Nach dem ersten Bogen versucht der Planer, die verbliebene Folge direkt an den
beobachteten Radius anzupassen. Jede Uebernahme hat eine komplette Kollisionspruefung.
Die folgenden Startmarkierungen bleiben gleich.
Das sind Modellabstände, keine unabhängig gemessenen realen Reserven. Bereits vollständig
enthaltene, aber vorne knappe Positionen werden nachkorrigiert. Der hintere ToF
kann während der normalen Bremsintervalle eine passende rosa Referenz liefern;
bei unpassendem Winkel oder ungültigen Daten bleibt die geometrische Prüfung
massgebend. Die folgenden **Startmarkierungen bleiben gleich**.

## Aufstellung

Die beiden rosa Begrenzungen und die Aussenwand bleiben in der normalen
Parkfeldaufstellung. Entferne Hindernisse aus dem unmittelbaren Manövrierraum:
Dieser isolierte Test besitzt keine eingelernte Hinderniskarte.

| Was messen? | Sollmass |
| --- | ---: |
| Freie Länge zwischen den beiden rosa Innenflächen | **24,75 cm** |
| Länge der rosa Begrenzungen von der Aussenwand ins Feld | **20,00 cm** |
| Hinterachsmitte bis zur Aussenwand, senkrecht gemessen | **29,46 cm** |
| Alternativ: Hinterachsmitte bis zur gedachten Linie zwischen den rosa Spitzen | **9,46 cm** auf der normalen Fahrbahnseite |
| Hinterachse längs der Wand relativ zur nahen rosa **Innenfläche** | **2,28 cm ausserhalb der Parklücke**, in Richtung Fahrzeugfront |
| Fahrzeugausrichtung | Parallel zur Aussenwand, Vorderräder gerade |

Hinterachsmitte bedeutet die Mitte der Achse zwischen den beiden Hinterrädern,
nicht die Vorderkante oder Hinterkante des Gummireifens. Nutze zum Aufstellen
zwei Markierungen für die Hinterradmitten. Möglichst auf etwa2mm genau messen
und sorgfältig parallel ausrichten; die Firmware ermittelt diesen Startpunkt
in diesem Modus **nicht selbst**. Diese Aufstellgenauigkeit ist eine
Vorbereitungsvorgabe, keine nachgewiesene Fehlertoleranz.

Die nahe rosa Begrenzung ist diejenige, an deren Ende der Roboter steht.
Ihre Innenfläche zeigt zur freien Parklücke. Die Hinterachse liegt2,28cm
weiter ausserhalb der Lücke. Da die rosa Begrenzung selbst2cm dick ist,
liegt die Hinterachse damit nur etwa2,8mm jenseits ihrer äusseren Seitenfläche.
Das zur Wand gerichtete Hinterrad hat bei gerader Aufstellung etwa3,21cm
Abstand zur gedachten Linie durch die rosa Spitzen.

```text
Blick von oben; Aussenwand unten, Fahrbahn oben

CW:   <--- FRONT -- HINTERACHSE     | freie Parklücke |
                          2,28 cm --> nahe Innenfläche

CCW:              | freie Parklücke |     HINTERACHSE -- FRONT --->
                              nahe Innenfläche <-- 2,28 cm

Beide Richtungen: Hinterachse29,46cm vor der Aussenwand;
die beiden rosa Begrenzungen reichen20cm von der Wand ins Feld.
```

## Start und Wiederholung

1. M7 hochladen, Fahrfreigabe ausschalten und den Roboter wie oben aufstellen.
2. Bei ausgeschalteter Fahrfreigabe im seriellen Terminal `park cw` oder
   `park ccw` senden. Reset allein wählt den vollständigen Hindernislauf.
3. Blau bedeutet ausgewählter Modus bereit. USB-Kabel zum Fahren abziehen und
   mit der normalen Fahrfreigabe starten. Er beginnt nach der Brems- und
   Lenkwartezeit direkt mit der Einparkbewegung.
4. Nach Erfolg hält er geparkt und speichert automatisch. Bei Abbruch normal
   speichern. Vor dem nächsten Versuch Fahrfreigabe aus und wieder genau an
   die Startmarkierungen stellen.
5. Für jeden neuen Parkversuch erneut `park cw` oder `park ccw` auswählen.
   Ein unterbrochener Versuch setzt sich durch blosses Wiedereinschalten
   nicht aus einer beliebigen Zwischenposition fort.

`park stop` beendet diesen Modus. Normales Speichern und USB-Logformat bleiben
erhalten. Beim Melden bitte Lognummer, CW/CCW und die Phase eines eventuellen
Kontakts oder manuellen Stopps nennen.

## Zurück zum vollständigen Parcours

Nach dem Hochladen gilt wieder `STARTUP_ROBOT_MODE=MODE_OBSTACLE_CHALLENGE`:
Reset wählt den vollständigen Hindernislauf mit Ausparken, drei Runden und
Einparken. Alternativ bei ausgeschalteter Fahrfreigabe `O` senden. Ein neuer
Build nur zum Wechseln zwischen normalem Lauf und Parktraining ist nicht nötig.
Die bestehende ältere Option `OBSTACLE_FINAL_PARKING_PRACTICE_ENABLED=false`
betrifft einen anderen Test mit vollständiger Anfahrt; sie bleibt aus.

Der isolierte Versuch prüft das Einparken ab definierter Position. Er bestätigt
nicht die vorherige automatische Anfahrt und Parkfeldlokalisierung über drei
Runden. Modellprüfungen ersetzen die jetzt vorgesehenen realen Parkversuche
nicht.


## Stand 2026-11-01: kuerzere Einparkfolgen

Revision `20261101_efficient_entry` prueft zuerst eine sichere Folge mit zwei
Rueckwaertsbogen. Wenn diese nicht passt, bleiben die geprueften Rangierfolgen.
Freier Weg wird aus Pose und Fahrzeugumriss berechnet; kein fester6cm-Zug.
Verbleibende Zuege werden nach Messabweichungen frisch geprueft und angepasst.
Details und Validierung: [Effiziente Einfahrt](EFFICIENT_PARKING_ENTRY_20261101.md).
