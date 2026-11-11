# Surprise Rule: Erkennung und gespeicherte Umfahrseite

Nachprüfung desselben Tages: siehe
[erste Runde und Parken](LAP1_PARKING_RECHECK_20261111.md). Sie ergänzt die
Übergabe einer unbrauchbaren Primärhöhe an eine brauchbare Ersatzmessung im
selben Bild; die Policyprüfung umfasst damit24statt21Fälle. Der Build unten
dokumentiert den ersten Umsetzungsstand; der aktuelle Build steht im Folgebericht.

Stand: 11. November 2026. Software und M7-Build geprüft; echte50/150-mm-Pfosten
sind mit der montierten Kamera noch nicht abgenommen. Keine Firmwareübertragung
oder Roboterbewegung in dieser Sitzung.

## Verhalten

Die Teilnahme ist vor dem Lauf festgelegt. In `include/config.h` ist
`OBSTACLE_SURPRISE_RULE_ENABLED` standardmässig1. Für einen normalen Versuch
ohne Zusatzaufgabe vor dem Übersetzen auf0 setzen beziehungsweise als
Build-Define mit0 überschreiben. Während des Laufs wird nicht umgeschaltet.

Die Kamera behält die sichtbare Farbe. Nach drei unterschiedlichen,
geometrisch zugeordneten Bildern derselben Farbe und ungewöhnlichen Höhe wird
genau ein Platz als Sonderpfosten gespeichert. Seine wirksame Kartenfarbe ist
die ursprüngliche Farbe:

| Sichtbar und Höhe | Gespeicherte ursprüngliche Farbe | Vorbeifahrt |
| --- | --- | --- |
| GREEN50/150mm | RED | rechts |
| RED50/150mm | GREEN | links |
| Normal100mm | sichtbare Farbe | bisherige Seite |

Die gespeicherte Platzidentität verhindert eine zweite Umkehrung. Die bestehende
Karte verwendet die ursprüngliche Farbe für die erste Umfahrung und die
Routen der Runden2/3. Nach Runde1 bleibt sie wie bisher eingefroren.

Der **erste tatsächlich angetroffene Pfosten** ist ausgeschlossen. Ein zweiter
Pfosten im Startabschnitt ist laut Zusatzregel zulässig. Die Entscheidung
verwendet deshalb die Reihenfolge der Plätze entlang der Fahrtrichtung,
bestätigte frühere Pfosten sowie bereits geklärte leere Plätze. Eine frühere
Kamerasichtung eines weiter entfernten Pfostens genügt nicht als Reihenfolge.
Gespeicherte Plätze hinter dem Start werden gesondert berücksichtigt.

## Geometrie und Unsicherheit

Die Distanz kommt weiterhin vom Bodenpunkt des Pfostens. Bei aktivierter
Zusatzregel entfällt die Ersatzrechnung mit angenommener100-mm-Höhe, wenn kein
brauchbarer Bodenpunkt vorliegt. Die bestehende Boden-Kalibrierung und die
HSV-Farbgrenzen wurden nicht neu abgestimmt.

Die bisherige untere Bildregion bleibt erhalten. Ergänzend verfolgt die Kamera
einen schmalen Pfostenstamm nach oben, damit150-mm-Pfosten nicht allein wegen
der bisherigen oberen Schnittkante wie100-mm-Pfosten aussehen. Grundbreite,
Bildrand, Bodenpunkt und seitliche Begrenzung werden geprüft. Ein verbundenes
schmales Hintergrundband darf die Messung nicht unbemerkt in eine andere
Höhenklasse verschieben; breite Raumflächen ergeben keine gesicherte Höhe.

Für Bodenpunkt `b`, obere Kante `t`, Horizont `h`, Hauptpunkt `cy`, vertikale
Brennweite `f` und Bodenfaktor `S` gilt im verwendeten Lochkameramodell:

```
q = (cy - h) / f
Kamerahöhe = S / (f * (1 + q*q))
Pfostenhöhe = Kamerahöhe * (b - t) / (b - h)
              * (1 + q*q) / (1 + q*(cy - t)/f)
```

Die vertikale Brennweite wird aus dem bestehenden horizontalen Modell bei
quadratischen Pixeln übernommen. Das ist eine zu prüfende Modellannahme.
Höhenfenster:32..68mm,80..120mm,132..178mm; dazwischen bleibt die Höhe unbekannt.
Bei abgeschnittener Oberkante ist nur eine ausreichend starke150-mm-Untergrenze
zulässig. Wechselnde Höhe/Farbe, abgelaufene Stimmen oder wiederholte Aufrufe
für dasselbe Bild ergeben keine drei unabhängigen Bestätigungen.

Ein späterer Pfosten unbekannter Höhe wird vor der Sonderpfostenfestlegung
nicht vorschnell als gewöhnlich bestätigt. Die bestehende Sichtprüfung und
Wiederanfahrt müssen eine bessere Beobachtung liefern. Ein sichtbares Objekt
mit unklarer Höhe ist kein Leerplatznachweis. Es wurden keine zusätzlichen
Fahrmanöver, neuen Geschwindigkeitsgrenzen oder Parkbewegungen eingeführt.
Die GREEN-Ersatzsuche durchläuft dieselbe Höhenentscheidung; gepufferte
Ausparkbilder bleiben lediglich vorläufige Hinweise und umgehen sie nicht.

## Bildfreigabe

Der Nutzer hat `20261027_green_400mm_01.png` ausdrücklich freigegeben. Im Replay
wird GREEN gefunden und als normal100mm eingeordnet: Modellhöhe92.9173mm,
Bodenpunkt138, Distanz390.7mm bei historisch angegebenen400mm. Das ist ein
einzelner stationärer Normalpfosten, kein Nachweis für beide Sonderhöhen.

`20261026_green_new_pose_400mm_01` ist wegen zyklisch versetzter Bildteile
**ungültig**. `20261026_green_final_400mm_01` wurde ebenfalls ausdrücklich
**nicht zur Verwendung freigegeben**. Alle Originalbytes bleiben erhalten.
`simulation/evidence/camera_diagnostics/validation_status.json` enthält Status
und SHA-256 der vier jeweiligen Begleitdateien. Nicht aufgeführte Bilder sind
ungeprüft. Vor neuer Verwendung muss der Nutzer das konkrete Bild bestätigen.

Die neue Höhenprüfung, die GREEN-Sichtprüfung und das Kamera-Replay verwenden
für neue Akzeptanz nur explizit freigegebene, hashgeprüfte Bilder. Frühere
allgemeine Oct26-Replayzahlen gelten nicht als Abnahme dieser neuen Logik.
Die historischen Berichte sind entsprechend gekennzeichnet.

## Prüfungen

- `check_surprise_rule.py`:648 unabhängig projizierte RGB565-Bilder mit beiden
  Farben,50/100/150mm,300..600mm, drei Bildpositionen, drei Sockelansichten und
  drei Kamerahöhen;48 Zuordnungs-/Bestätigungsfälle CW/CCW;21 Sonderregel- und
  Fehlerfälle. Alle bestanden. Die tatsächliche Bildverarbeitung und
  Zuordnungsentscheidung werden kompiliert; die nachfolgende Kartenmutation
  ist in diesem Fixture ersetzt.
- Freigegebenes reales Normalbild: erkannt, keine falsche Farbverkehrung.
- Bisherige Akquisition ohne Zusatzaufgabe:52 Fälle; GREEN-Ersatzsuche7 Fälle.
- Ecke565:10 Evidenzfälle und27 Wiederanschlüsse; Negativkontrolle reproduziert
  das alte Problem. Ecke576: direkter Nachweis/Scout und Negativkontrolle.
- Leerplatz-/Belegungsschutz589/621 sowie synthetische GREEN-/RED-/Grauflächen,
  Bodenlinien und Hintergrundbänder bestehen. Abgelehnte Archivbilder werden
  übersprungen.
- Spätere Routen:2807 Fälle, keine Fehler. Dies prüft Modellrouten und
  Zustandsverhalten, keine reale Reifenhaftung oder Kameraaufnahme während Fahrt.

IDE-PlatformIO `giga_r1_m7`: **SUCCESS**, RAM484896/523624B,
Flash595392/786432B. Firmware598944B, SHA-256:
`1b4d87efb71b9b36f883b849c9e0bb5e795346526437358a85a54b0bd06f39ba`.
M4 verbraucht keine geänderte Schnittstelle und wurde nicht gebaut. Vorhandene
Serial-/ungenutzte-Legacy-Warnungen bleiben; kein Übersetzungsfehler.

Nächster notwendiger Nachweis: echte50/100/150-mm-Pfosten mit unveränderter
Kameramontage und normalem Parcourslicht sowie CW/CCW-Durchfahrten. Vor weiteren
Bildauswertungen Bildfreigabe einholen; neue vollständige Testlogs zuerst
sichern und hashen. Die Modelltests begründen keine100-Prozent-Zuverlässigkeit.

Regelquelle: [offizielle Zusatzaufgabe, Version9. November2026](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf).
