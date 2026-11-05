# Nur das Rückwärts-Einparken üben

Die aktuelle Version startet nach Reset wieder im vollständigen
`OBSTACLE_CHALLENGE`. Für das separate Training bei ausgeschalteter Fahrfreigabe
`park cw` oder `park ccw` auswählen. Im Parktraining: keine Ausparkbewegung, keine Runden und keine Anfahrt
in die entfernte Ecke. Sie beginnt am Übergabepunkt direkt vor dem
Rückwärts-Einparkmanöver und benutzt danach denselben Einparkplaner wie der
normale Hindernislauf, einschliesslich Endkorrektur und Halten nach Abschluss.
Die Kamera wird für diesen isolierten Modus nicht initialisiert.

Stand 2026-11-05, nach Logs651-656: Gesamtbreite inklusive Raeder123mm,
physischer Achsabstand98mm, Heckueberhang35mm. Die Front reicht ab Hinterachse
123mm gerade und135mm bei Volleinschlag vor. Diese Lenkkontur wird auch in
engen Rettungswegen geprueft; zusaetzlich bevorzugt die Planung4mm Frontreserve.
Der kalibrierte kinematische Radstand127mm bleibt davon getrennt.

Der Einfahrpunkt bleibt laengs10mm Richtung Heck verschoben, lokalX260,26mm.
Er liegt jetzt naeher an den rosa Spitzen: Hinterachse271,5mm ab Aussenwand,
also10mm zwischen geradem Rad und Spitzenlinie. Bevorzugte Endachslage128,5mm
ab Wand: nominal10mm zwischen Fahrzeugkante und offener Parkkante.
Sicher tiefere Fortsetzungen bleiben fuer bereits schraege Zwischenlagen moeglich.

Die Endposition wird nicht auf ein laengs festes Zentrum gezwungen. Sobald er
parallel, vollstaendig enthalten und sicher stillsteht, beendet er das Parken.
Er faehrt danach nicht zur Mitte zurueck. Der hintere ToF wird waehrend normaler
Bremsintervalle verwendet, wenn Winkel und Messqualitaet die Referenz zulassen.
Normale kurze Folgen haben Vorrang. Erst wenn diese vollstaendig geprueft nicht
passen, sucht er begrenzt zusaetzliche Rangierzuege mit verfuegbarem Platz an beiden
Enden. Jede uebernommene Folge erhaelt eine vollstaendige Kollisionspruefung.
Das sind Modellreserven; die neue Version muss auf dem Fahrzeug bestaetigt werden.

## Aufstellung

Die beiden rosa Begrenzungen und die Aussenwand bleiben in der normalen
Parkfeldaufstellung. Entferne Hindernisse aus dem unmittelbaren Manövrierraum:
Dieser isolierte Test besitzt keine eingelernte Hinderniskarte.

| Was messen? | Sollmass |
| --- | ---: |
| Freie Länge zwischen den beiden rosa Innenflächen | **24,75 cm** |
| Länge der rosa Begrenzungen von der Aussenwand ins Feld | **20,00 cm** |
| Hinterachsmitte bis zur Aussenwand, senkrecht gemessen | **27,15 cm** |
| Alternativ: Hinterachsmitte bis zur gedachten Linie zwischen den rosa Spitzen | **7,15 cm** auf der normalen Fahrbahnseite |
| Hinterachse längs der Wand relativ zur nahen rosa **Innenfläche** | **1,28 cm ausserhalb der Parklücke**, in Richtung Fahrzeugfront |
| Fahrzeugausrichtung | Parallel zur Aussenwand, Vorderräder gerade |

Hinterachsmitte bedeutet die Mitte der Achse zwischen den beiden Hinterrädern,
nicht die Vorderkante oder Hinterkante des Gummireifens. Nutze zum Aufstellen
zwei Markierungen für die Hinterradmitten. Möglichst auf etwa2mm genau messen
und sorgfältig parallel ausrichten; die Firmware ermittelt diesen Startpunkt
in diesem Modus **nicht selbst**. Diese Aufstellgenauigkeit ist eine
Vorbereitungsvorgabe, keine nachgewiesene Fehlertoleranz.

Die nahe rosa Begrenzung ist diejenige, an deren Ende der Roboter steht.
Ihre Innenfläche zeigt zur freien Parklücke. Die Hinterachse liegt1,28cm
weiter ausserhalb der Lücke. Da die rosa Begrenzung selbst2cm dick ist,
liegt die Hinterachse damit nur etwa2,8mm jenseits ihrer äusseren Seitenfläche.
Das zur Wand gerichtete Hinterrad hat bei gerader Aufstellung 10mm
Abstand zur gedachten Linie durch die rosa Spitzen.

```text
Blick von oben; Aussenwand unten, Fahrbahn oben

CW:   <--- FRONT -- HINTERACHSE     | freie Parklücke |
                          1,28 cm --> nahe Innenfläche

CCW:              | freie Parklücke |     HINTERACHSE -- FRONT --->
                              nahe Innenfläche <-- 1,28 cm

Beide Richtungen: Hinterachse27,15cm vor der Aussenwand;
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


## Abschluss ohne Zentrieren (2026-11-05)

Die um10mm verschobene Einfahrt wird gegen die echten, unveraenderten rosa
Begrenzungen geprueft. Der Planer darf deshalb bei engem Platz eine andere
sichere Bogenfolge waehlen; eine starre Verschiebung jeder Bewegung waere
nicht fuer jede gemessene Luecke kollisionsfrei.

Parallel bedeutet innerhalb der bestehenden2Grad-Toleranz. Der gesamte
Roboter muss im Parkfeld liegen; beim fruehen Stoppen aus laufender Bewegung
bleiben zusaetzlich3mm Modellreserve fuer Registrierung/Rundung. Nach dem
Bremsen gelten die vorhandene5mm-Endlagenpruefung und eine sichere Stellung
der geraden Vorderraeder. Die frueher bevorzugten30mm Platz vorne erzwingen
keine anschliessende Zentrierfahrt mehr. Ein unsicherer oder noch schraeger
Stand wird weiterhin korrigiert.
