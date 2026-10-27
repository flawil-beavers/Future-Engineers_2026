# Korrekturen nach der Codeprüfung, 27.10.2026

Ausgangsbefund: [FULL_CODE_REVIEW_20261027.md](FULL_CODE_REVIEW_20261027.md).
Die nachfolgend belegten Fehler wurden gezielt korrigiert. Keine neuen
physischen Läufe oder Original-Logs liegen vor; die vier zuletzt beschriebenen
CW-Läufe bleiben für die spätere Ursachenprüfung offen. Kein Upload durch den
Agenten.

## Verhalten der korrigierten Version

| Bereich | Korrektur |
| --- | --- |
| Lenkung nach Gyro-Pause | Akzeptierte gerade und zusätzliche gebogene Beobachtungsrückfahrten stellen die Lenkungsfreigabe wieder her. Gleiches gilt für die vier bewegten Open-Navigationszustände. Verwerfende Geometrie-/Gesundheitsprüfungen bleiben wirksam. |
| Zeitgrenzen der Beobachtungsrückfahrt | Eine tatsächliche Gyro-Pause wird einmalig aus dem Phasenzeitkonto herausgerechnet; auch beim Transportneustart beginnt diese Pause vor der blockierenden Sensorinitialisierung. |
| Motorprofil nach Wiederanlauf | Vorher vergangene Zeit erzeugt keinen rückwirkenden neuen Sollweg. Ein Neustart beginnt bei null; nach einer aktiven Schleifenlücke über 100 ms wird auf gemessene Bewegung und aktuelle Encoderposition zurückgesetzt. Keine zusätzliche dauerhafte Stoppsperre. |
| USB und neuer Start | Ein angeforderter Start wartet auf den alten Speichervorgang. LOW oder explizites Stoppen widerruft diesen Start. Während geschwindigkeitsgeregelter Fahrt führt der Logger keine Dateioperationen aus. |
| USB-Dateisuche und Datenerhalt | Höchstens vier Dateinamenproben je Update; keine unbegrenzte Suche in einem Tick. Ein fehlgeschlagener Speicherversuch schützt seinen RAM-Inhalt auch vor `clear()` beim nächsten Lauf. |
| M4-Software-I2C | Daten- und ACK-Taktfehler werden als fehlerhafte Transaktion gemeldet; kein teilweise gelesenes Byte und kein alter Teilpuffer wird als vollständiger Registerinhalt geliefert. |
| ToF-Herstellertreiber beider Kerne | Endlosschleifen bei Registeradressierung und leeren grossen Leseabschnitten entfernt. Höchstens zwei Adressierungsversuche; ein einzelner vorübergehender NACK kann sich weiter erholen. Leere/kurze Transfers und auch Fehler mittlerer Schreibabschnitte gehen an die vorhandene Sensorfehlerbehandlung. |
| Open-Eckbestätigung | Jede Seite zählt nur eigene neue und gültige ToF-Sequenzen. Die Richtungswahl benutzt die tatsächlich bestätigte Seite. Wiederholtes Lesen desselben Frames ist keine Mehrfachbestätigung. |
| Optionale Innenlinie Runde 2/3 | Kandidaten werden über zwei Umläufe mit derselben endlichen Motorrampe wie im Fahrcontroller geprüft, statt nur mit konstanter räumlicher Schrittweite. Unsichere oder nicht innerhalb des Zeitbudgets fertig geprüfte Kandidaten fallen auf die vorherige Form zurück. |

Die maximale Geschwindigkeitsanforderung bleibt **520 mm/s**. Normale
Beschleunigungs-/Jerkparameter bleiben gleich. Für spätere Runden wächst die
vorweggenommene Reaktionszeit vor Kurven von 0.35 auf **0.40 s**; das entspricht
bei 520 mm/s und der bestehenden 10-%-Geschwindigkeitszugabe höchstens
28.6 mm zusätzlicher Bremsvorausschau. Damit wird auch der vorbestehende
Dynamikfall 154 abgefangen. Es gibt keine pauschale neue Geradendrosselung.

Die optionale Kandidatenprüfung behält nominalen und gekoppelt gestörte Starts
(±10 mm/±1°, dort zusätzlich 10 % Geschwindigkeitszugabe), 40 mm modellierte
Wandreserve und die bestehenden Karosserieprüfungen. Für neue Kandidaten
werden 32 statt 30 mm Hindernisreserve verlangt, um kleine Änderungen durch die
Zeitschrittauflösung abzufangen. Die allgemeine dynamische Akzeptanzgrenze
bleibt 30 mm und 0.8 m/s². Der kooperative 30-ms-Suchzeitrahmen bleibt bestehen;
ein Rückfall setzt keine neue Fahrstörung.

Kamera-Farbschwellen, Sitzzuordnung, Erstlauf-Geometrie, Parksegmente,
Ausparksegmente und deren Geschwindigkeitswerte wurden nicht umgestimmt.
Unbenutzte Alt-Funktionen wurden nicht zusammen mit diesen Funktionskorrekturen
entfernt. Die im Review beschriebenen möglichen Live-DMA-/Gyro-Zeitstempelrisiken
bleiben als nicht physisch nachgewiesene Themen dokumentiert.

## Reproduzierbare Prüfungen

Die Skripte werden mit dem IDE-verwalteten Python aus dem Repository ausgeführt.
Alle generierten C++-Dateien, Binärdateien und Rohberichte bleiben in
`local_workspace/`.

| Prüfung | Ergebnis |
| --- | --- |
| `review_runtime_edges_20261027.py` | PASS: kein Sollwegsprung bei 215/819/1200 ms; gemessene aktive Bewegung und nächste normale Rampe; alle vier akzeptierten Rückfahrtbefehle nach Halt; verworfener Befehl bleibt aus; Startwarteschlange/LOW-Abbruch/einmaliger Start; 16 Datenbit- und zwei ACK-Timeouts plus Erholung; 534 Dateien in 134 Updates mit 535 Proben; fehlgeschlagener RAM-Snapshot bleibt erhalten; frische Sequenzen und Zeitüberlauf. |
| `check_drive_profile_timing.py` | **40 000** Vergleiche mit originalem `drive_loop()` aus `861ed8e`: bei 1/10/20/50 ms, wechselnden positiven/negativen Anforderungen und Beschleunigungsgrenzen 0/60/120/500/1000 **0 mm/s Geschwindigkeitsabweichung**. Auch die bisherige Arduino-`constrain`-Semantik der Kalibriergrenzen bleibt erhalten. |
| `check_bounded_tof_transport.py` | PASS: tatsächlich erzeugte I/O-Wrapper beider installierter Herstellertreiber; 96-Byte-Lesen/-Schreiben; dauerhafte und vorübergehende Adressfehler, leere Abschnitte, fehlende Bytes, Teil-Schreibvorgänge, mittlere Schreibfehler und Erholung beim nächsten Zugriff. |
| `check_runtime_safety.py` | Alle bisherigen Host-Prüfungen PASS, einschliesslich Gyro/ToF-Fehler und Erholung, Freigabeschalter, Parser, Pose, Vision, Dispatcher und Parkpausenzeiten. |
| `check_later_laps.py` | **2807/2807** Geometriefälle PASS. |
| `review_later_lap_matrix_20261027.py` | **2191/2191** bei 20 ms PASS, ebenso die Kontrollvariante ohne neue Innensuche. Vorher 7 gegen 1 Grenzverletzung; jetzt jeweils 0. |
| `check_later_lap_dynamics.py --full-matrix` | **6573/6573** dynamische Fälle PASS: dieselben 2191 ungekappten Karten jeweils bei 10, 20 und 50 ms. Siehe Minima unten. |
| `check_later_lap_inner_routes.py` | PASS: 336 Vergleiche der schnellen mit der ursprünglichen vollständigen Geometrieprüfung, vier Budget-/Zeitüberlauffälle und acht gezielte Übergänge je Vergleichsvariante. Die gezielte innere Weiterfahrt spart weiterhin 118.92 mm und 0.562 s berechnete Zielzeit je Runde, CW und gespiegelt. |
| `check_first_lap_geometry.py` | **1736/1736** PASS; modellierte Hindernisminima CW 47.657 mm, CCW 47.6197 mm. Dies ersetzt keinen Kameratest. |
| `check_final_parking_robustness.py` | **1540** Szenarien ohne unsicheres Modellresultat; alle **990** bekannten regulären Anfahrten parken. Nicht geeignete fremde/fehlerhafte Ausgangslagen werden teilweise oder vollständig abgelehnt. Zusätzlich 46 Grenz- und fünf Messqualitätsprüfungen PASS. |
| `check_parking_segment_sensor_resume.py` | **42** Prüfungen aller sieben Parksegmente in beiden Richtungen PASS; negative Kontrolle reproduziert den alten Fehler. |
| `check_gyro_path_servo_resume.py`, `check_connector_servo_resume.py` | PASS, einschliesslich negativer Kontrollen mit entfernter Lenkungsfreigabe. |

| Regelschritt im dynamischen Gesamtmodell | Fälle | Höchste Querbeschleunigung, m/s² | Kleinster Hindernisabstand, mm |
| --- | --- | --- | --- |
| 10 ms | 2191 | 0.760805 | 32.1480 |
| 20 ms | 2191 | 0.768285 | 32.2451 |
| 50 ms | 2191 | 0.796430 | 33.4597 |

Alle sieben vorher im Review festgehaltenen Karten bleiben dort als genaue
Sitzarrays erhalten und sind Teil dieser Gesamtmatrix. Die Lösung enthält
keine Sonderbehandlung ihrer Fallnummern.

## ToF-Anpassung im Build

`platformio.ini` lädt `scripts/bounded_tof_transport.py` als Pre-Build-Skript
für beide Kerne. Das installierte PlatformIO-Middleware-API ersetzt ausschliesslich
die zwei I/O-Funktionskörper der lokal installierten VL53L4CX-Bibliothek durch
Aufrufe aus `include/bounded_tof_transport.h`. Der übrige Herstellercode und
sein Lizenztext bleiben erhalten. Die Bibliotheksdateien und Paket-Caches
werden nicht verändert. Erzeugte Quellen liegen in
`local_workspace/generated_tof/<environment>/`; Objekte bleiben im normalen
`.pio/build/<environment>/`.

Die korrigierte Profil-Hilfsdatei bleibt auch mit getrennten Referenzkonstanten
im bestehenden Messwerkzeug vergleichbar. Der zusätzliche Sieben-Karten-Vergleich
mit gleicher neuer Bremsplanung auf beiden Seiten besteht: das CW-Beispiel A
bleibt 405.27 mm pro Runde kürzer (berechnete Zielzeit 16.9407 → 15.4049 s),
das gespiegelte CCW-Beispiel C 193.27 mm (16.5581 → 14.9777 s). Das sind
Zielzeiten im Modell, keine Prognose einer gemessenen Gesamtlaufzeit.

Ein unerwartet geänderter Herstellertreiber führt zu einer verständlichen
Buildfehlermeldung statt einer stillschweigend fehlenden Anpassung. Die
Bibliotheksauflösung/Frameworkversion wurde nicht aktualisiert. Bounded bedeutet
hier eine begrenzte Anzahl unserer Busaufrufe; die interne Laufzeit eines
einzelnen hardwareabhängigen `Wire`-Aufrufs wird dadurch nicht bewiesen.

## Firmware und verbleibende praktische Abnahme

Beide finalen IDE-PlatformIO-Builds **SUCCESS**, ohne Clean oder separaten Cache:

| Kern | RAM | Flash |
| --- | --- | --- |
| `giga_r1_m7` | 442408 / 523624 Bytes | 524296 / 786432 Bytes |
| `giga_r1_m4` | 59768 / 294248 Bytes | 155328 / 1048576 Bytes |

SHA-256 der Dateien `.pio/build/<environment>/firmware.bin`:

```text
giga_r1_m7 5542896c58ca0c36b2c389c0be2d63471b2a0dbeeb27a1cbedee4b44ab7b22d8
giga_r1_m4 fbc3da65845172d9be1405ec56b626da1b716999ce2fd31e591a4c3a14057215
```

Der Compiler meldet weiterhin vorhandene Makro- und Bibliothekswarnungen;
beide Builds haben erfolgreich kompiliert und gelinkt.

Beide Kerne sind diesmal betroffen: M7 durch Steuerung/Planung,
M4 durch Software-I2C und beide durch den begrenzten ToF-Transport. Protokoll und
RPC-Layout bleiben kompatibel. Der Benutzer spielt die Firmware selbst auf.

Die bestandenen Prüfungen sind Modell- und Hostnachweise. Servoverzug,
Reifenhaftung, Lichtverhältnisse, Montage und die tatsächliche Kamerapuffer-
Zeitlage sind keine daraus gemessenen Eigenschaften. Normale CW-/CCW-Läufe mit
dem eindeutig identifizierten Firmwarestand bleiben die praktische Abnahme;
keine spezielle ToF-Abtastfahrt, zusätzlichen Bewegungen oder Pausen verlangt.
Wenn die fehlenden Logs eintreffen: vollständige Originale zuerst unverändert
archivieren und hashen, danach konkrete GREEN-/Einparkursache zuordnen.
