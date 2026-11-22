# Nur das Rückwärts-Einparken üben

Normales `O`, `park cw`, `park ccw` und der CW-Eckentest benutzen dieselbe feste Zugfolge. CCW spiegelt die Lenkrichtung. Der Sollabstand zu den rosa Spitzen beträgt 3 cm. An der Einparkposition ist die Hinterkante auf Höhe der Innenfläche der einfahrseitigen Schranke. Bei höchstens 10° Abweichung von der Parallelrichtung startet dort der erste Rückwärtszug nach dem Bremsen.

## CW-Eckentest mit fester Zugfolge

Dieser Versuch startet in der Ecke auf derselben Geraden wie die Parklücke. Die Front zeigt zur Parklücke, die Aussenbande neben der Parklücke liegt links. Das Heck steht **ca. 2 cm vor der quer hinter ihm liegenden Wand**. Die Mitte der Hinterachse steht **ca. 29,2 cm von der seitlichen Aussenbande** entfernt. So beträgt der Sollabstand der wandseitigen Radkante zur Linie durch die rosa Spitzen bei der Anfahrt **3 cm**. Räder gerade, Fahrzeug parallel zur Seitenbande. Halte die Strecke zwischen Ecke und Parklücke für diesen Versuch frei.

1. `giga_r1_m7` hochladen. Bei ausgeschalteter Fahrfreigabe `park cw fixed` im seriellen Monitor senden.
2. USB-Kabel zum Fahren abziehen und mit der normalen Fahrfreigabe starten. Auf der freien Anfahrt fährt der Roboter bis **145 mm/s**. Für die Schrankenmessung fährt er **66 mm/s** und bei dicht aufeinanderfolgenden ToF-Messungen bis **84 mm/s**. Vor der Einparkposition bremst er ab. Bei bis zu **10° Abweichung von der Parallelrichtung** beginnt dort nach dem Stillstand der erste Rückwärtszug.
3. Die vier Züge laufen in fester Reihenfolge. Rückwärts mit Volleinschlag bis zum Gyro-Ziel von 47°, gerade 12,5 cm zurück, rückwärts mit Gegenlenkung bis 22,5° Restwinkel, dann mit Gegenlenkung kurz vorwärts bis 0°.
4. Nach dem vierten Zug hält der Roboter an, prüft die Endlage und speichert Winkel und Encoderstrecken. Die Meldung `final_parking_stop` bestätigt die anhand der geschätzten Pose geprüfte Endlage. Die tatsächliche Freigängigkeit ist auf der Matte zu prüfen. Für einen neuen Versuch wieder in die Ecke stellen und den Befehl neu senden.

Die feste Zugfolge benutzt Gyro und Encoder. Eine frische hintere ToF-Messung von 20 mm oder weniger stoppt den Rückwärtszug. Die Brems-, Fortschritts-, Weg- und Zeitgrenzen bleiben aktiv. Nach der Endprüfung folgt keine weitere Fahrbewegung.

## Feste Zugfolge ab Parklücke

Bei ausgeschalteter Fahrfreigabe `park cw` oder `park ccw` auswählen. Der Roboter startet an der unten beschriebenen Einparkposition direkt mit denselben vier Zügen wie im normalen Lauf. In diesem Modus werden Aufstellung und freie Lückenlänge als Nennwerte übernommen. Die Kamera wird nicht initialisiert.

## Aufstellung

Die beiden rosa Begrenzungen und die Aussenwand bleiben in der normalen
Parkfeldaufstellung. Entferne Hindernisse aus dem unmittelbaren Manövrierraum:
Dieser isolierte Test besitzt keine eingelernte Hinderniskarte.

| Was messen? | Sollmass |
| --- | ---: |
| Freie Länge zwischen den beiden rosa Innenflächen | **24,75 cm** |
| Länge der rosa Begrenzungen von der Aussenwand ins Feld | **20,00 cm** |
| Hinterachsmitte bis zur Aussenwand, senkrecht gemessen | **29,15 cm** |
| Alternativ: Hinterachsmitte bis zur gedachten Linie zwischen den rosa Spitzen | **9,15 cm** auf der normalen Fahrbahnseite |
| Hinterachse längs der Wand relativ zur nahen rosa **Innenfläche** | **3,50 cm ausserhalb der Parklücke**, in Richtung Fahrzeugfront |
| Fahrzeugausrichtung | Parallel zur Aussenwand, Vorderräder gerade |

Hinterachsmitte bedeutet die Mitte der Achse zwischen den beiden Hinterrädern,
nicht die Vorderkante oder Hinterkante des Gummireifens. Nutze zum Aufstellen
zwei Markierungen für die Hinterradmitten. Möglichst auf etwa2mm genau messen
und sorgfältig parallel ausrichten; die Firmware ermittelt diesen Startpunkt
in diesem Modus **nicht selbst**. Diese Aufstellgenauigkeit ist eine
Vorbereitungsvorgabe, keine nachgewiesene Fehlertoleranz.

Die nahe rosa Begrenzung ist diejenige, an deren Spitze der Roboter ansetzt. Von ihrer zur Parklücke gerichteten Innenfläche aus misst du **3,50 cm in Richtung Fahrzeugfront** bis zur Hinterachsmitte. Das entspricht dem Hecküberhang. Bei geraden Rädern liegt die wandseitige Radkante 3,00 cm von der Linie durch die rosa Spitzen entfernt.

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


Frühere Planungsstände sind im [Archiv](../docs/history/README.md) dokumentiert.
