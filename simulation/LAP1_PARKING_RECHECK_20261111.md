# Nachprüfung: erste Runde und Parken

Stand11. November2026. Mehrere konkrete Stillstandsursachen behoben. Die
normalen geprüften Fälle bestehen; eine bedingungslose Aussage „bleibt niemals
stehen und parkt aus jeder Position“ wäre weiterhin falsch. Es gibt noch
unaufgelöste Fälle am engen Ende der zugelassenen Messspanne, siehe unten.
Keine neue USB-Quelle verfügbar, keine neuen physischen Läufe, kein Upload,
Reset oder Motorstart. Alle Ergebnisse dieser Sitzung sind Softwareprüfungen.

## Erste Runde

1. **Unbrauchbare Primärhöhe blockierte die bessere Ersatzmessung.** Die neue
   Sonderregel hatte bereits bei unbekannter Höhe das Kamerabild als verbraucht
   markiert. Eine im selben Bild brauchbare, enger begrenzte GREEN-Ersatzmessung
   konnte dann nie abstimmen. Unbrauchbare Höhen verbrauchen jetzt keine Stimme.
   Eine brauchbare Messung desselben Bildes zählt genau einmal. Der vorhandene
   400-ms-Stimmenzeitraum und die Anforderungen an Farb-/Platzzuordnung bleiben.
   Der erste tatsächlich angetroffene Pfosten braucht weiterhin keine Höhe.

2. **Begrenzte Abweichung wurde zum dauerhaften Fehlerhalt.** Bei einer
   geraden Suchfahrt führten bereits mehr als3Grad oder15mm Querabweichung
   in einen Zustand ohne weitere Bildauswertung. Bei endlicher Pose und
   höchstens12Grad/25mm wird jetzt zuerst gebremst. Nach der bestehenden
   300-ms-Bremsphase wird die tatsächliche Position zur neuen Referenz;
   danach folgen neue Kamerabilder und vollständig neu geprüfte Blickwege.
   Das erweitert keine Fahrfreigabe für den alten, abweichenden Weg.

3. **Verworfene Suchbewegung darf weitere Sichtprüfung nicht sperren.** Ein
   nach dem Einschlagen nicht mehr zulässiger Bogen, ein unsicher gewordener
   Rückweg oder ein abgelehntes gerades Fahrkommando führen ebenfalls zur
   gebremsten Neuaufnahme. Es gibt keine blinde Rückkehr zum alten Ursprung.
   Frühere bestätigte Pfosten und tatsächlich geklärte Plätze bleiben erhalten.

4. **Fehlende Primärbestätigung nach dem Ausparken** löst auch in drei
   defensiven Übergängen ohne zuvor gesetztes Retry-Flag einen neuen
   Beobachtungsversuch aus. Sie setzt nicht mehr allein deswegen den
   permanenten Test-Halt.

UNKNOWN allein schaltet somit in diesen korrigierten Zweigen keine dauerhafte
Störung ein. Er liest weiter, prüft Rücksetzen und die vorhandenen beiden
Parallaxebögen und fährt erst nach Bestätigung sowie geprüftem Anschluss weiter.
Kann aktuell kein sicherer Blickweg berechnet werden, bleibt die Bildauswertung
aktiv und prüft erneut; das bedeutet nicht zwangsläufig körperliche Bewegung.
Ungültige Pose, falsche Fahrtrichtung, starke Abweichung, überschrittene
Brems-/Weggrenzen und echte Sensorprobleme bleiben Sicherheitsstopps.
Die Startbewegungen behalten ihre Prüfung gegen die rosa Begrenzungen.

## Parken

**Zusammengesetzter Rettungsweg:** Vorwärts- und Rückwärtsstücke mit gleichem
Lenkwinkel können beim Zusammenbauen teilweise verschwinden. Der neue erste
Bogen hatte dann eine andere Anfangslage als die zuvor geprüfte Teilfolge.
Die reine Erzeugung der Endpunkte übersprang dessen Radius-Toleranzprüfung.
Das konnte denselben Plan ohne Fahrfortschritt bis `entry_no_progress`
wiederholen. Jetzt werden die ersten maximal15mm für beide Radiusgrenzen
gesondert geprüft; danach bleibt die komplette1-mm-Kollisionsprüfung erhalten.
Dieser kleine Zusatzcheck verbraucht kein bereits erschöpftes Suchbudget für
eine weitere vollständige Suche. Die bereits geprüften späteren Übergänge
behalten ihre Prüfung und Endlagen.

**Endpositionen in der Ersatzsuche:** Deren7-mm/4-Grad-Zellen waren gröber als
die Endbedingungen. Ein gültiger paralleler Endpunkt konnte deshalb als
Duplikat einer benachbarten, noch unzulässigen Lage verschwinden. Zulässige
Endpunkte werden jetzt gesondert behandelt und erhalten dieselbe vollständige
Endkonturprüfung wie andere Kandidaten. Der bestehende begrenzte512-Knoten-
Planer darf auch ausserhalb der rosa Spitzen bis lokalY320mm versuchen,
eine vollständige Folge zu finden. Speicher- und Suchbudgets bleiben begrenzt.

**Winzige Schlusskorrekturen und Bremsweg:** Nach gemessenem Nachlauf wird
für einen geplanten Endpunkt bis zu2mm zusätzliche Reserve eingeplant.
So wird keine Folge gewählt, die bei jedem kleinen Lenkwinkelüberschwingen
wieder knapp unter der Abschlussreserve landet. Die tatsächlich zulässige
parallele Endposition, der bevorzugte10-mm-Abstand zur offenen Parkkante und
der Stopp ohne Längszentrierung bleiben erhalten.

**Späte Geschwindigkeitsmeldung:** Ein einzelner noch nicht auf Stillstand
gefallener Wert beim Abschluss führt zur vorhandenen Brems-/Verifikationsphase
statt unmittelbar zu einem Parkabbruch. Ein dauerhaft nicht bremsender Roboter
bleibt ein echter Fehlerfall. Fertigmeldung erfordert weiterhin Stillstand,
Parallelität, vollständige Fahrzeugkontur im Feld und zulässige Neutralräder.

Fahrzeugmasse unverändert: Breite123mm, physischer Achsabstand98mm,
Heck35mm, Front ab Hinterachse123mm gerade/135mm eingeschlagen. Auch Tempo,
Parkziel, normale kurze Folge und Sicherheitsabstände wurden nicht reduziert.

## Ergebnisse und bekannte Grenze

| Prüfung | Ergebnis |
| --- | --- |
| Erste Runde, bekannte Karten/Geometrie | 868CW und868CCW bestanden; keine Kamerasimulation |
| Neue Neuaufnahme-Zustände | 29 bestanden: begrenzte Abweichung, weitere Bilder, sicherer Anschluss und echte Fehlerstopps |
| Discovery-Hold/Retry-Verteilung | 25 bestanden; aktueller Fixture-Kontext repariert |
| Startübergang576 | Scout/direkter Nachweis und CW/CCW-Primärretry bestanden; Negativkontrolle reproduziert alten Fehler |
| Aktive Parallaxensuche und gelöster Platz627 | Bestanden; kein ungeprüfter Wiederanschluss |
| Sonderregel | 648RGB565-Projektionen,48Zuordnungen,24Policyfälle bestanden, einschliesslich Ersatzmessung im gleichen Bild |
| Vollständiges Parken | 396/396 plus5Registrierungsfälle bestanden |
| Gemessene Geometrie,248.9/249.6mm Lücke | 24/24 bestanden, gespiegelt, Radius1..1.5, Anfangswinkel+/-1Grad |
| Zusätzlicher Bremsweg0/1.5/3mm | 84/84 plus8Sensor-/Gesundheitsfälle bestanden |
| Parallel anhalten | 16 bestanden |
| Neuer Übergabe-/Bremsfehler-Test | 6 bestanden; entfernte Anfangsbogenprüfung reproduziert beide `entry_no_progress`-Fälle |
| Breiter historischer Park-Stresstest | 332/350 geschafft;18geprüfte Abbrüche, keine Modellkollision |
| Aktuelle Anfahrposition, volle Messspanne | 96/120 geschafft;24Abbrüche ausschliesslich bei232.5mm Lücke |

Die letzte Prüfung benutzt die aktuelle Anfahrposition260.26/271.5mm,
Anfangswinkel+/-1Grad, beide Richtungen, Radiusfaktoren0.95..1.5 und
Lücken232.5/242.5/247.5/252.5/262.5mm. Alle96Fälle ab242.5mm bestehen.
**Bei232.5mm findet der begrenzte Planer aus dieser Position keinen vollständigen
geprüften Weg und bricht ab.** Das liegt innerhalb der bisherigen tolerierten
Marker-Messspanne; diese Grenze wird ausdrücklich nicht als behoben ausgegeben.
Die gemeldeten tatsächlichen Lücken248.9/249.6mm sind separat oben geprüft.

Der historische350-Fälle-Test enthält ausserdem alte, weiter entfernte
Anfahrpositionen und schräge Zwischenlagen. Vorher:320Erfolge/30Fehler;
jetzt332/18,12Fälle zusätzlich gelöst und keine zuvor bestandenen Fälle
verschlechtert. Vergleichsbasis: `src/final_parking.cpp` aus
`eb80e12cfda6e9a59ffaa8383d94e2a99ee4b4fe`, bei identischem aktuellem Hostmodell.
Die verbleibenden18Abbrüche betreffen ebenfalls232.5mm. Das beweist weder
Unmöglichkeit eines anderen Weges noch ein vollständiges Suchverfahren.

Ein älterer Sammeltest `check_returned_611_615.py` besteht nach dem
Trackingteil dessen20Fälle, scheitert danach aber an seiner alten stationären
Wandreferenzannahme. Er wird nicht als bestandene Parkabnahme gezählt.
Andere ältere Sammeltests enthalten ebenfalls überholte feste Zielwerte.
Die oben aufgeführten aktuellen Zustands- und Geometrieprüfungen ersetzen
keine physische Kamera-/ToF-/Haftungsabnahme.

Wichtige reproduzierbare Befehle mit dem IDE-Python:

```
simulation/check_lap1_reobserve.py
simulation/check_discovery_recovery_state_521_522.py
simulation/check_surprise_rule.py
simulation/check_parking_recovery_handoff.py
simulation/check_parking_recovery_handoff.py --without-combined-envelope
simulation/check_final_parking_sequence.py
simulation/check_parking_measured_651_656.py
simulation/check_parking_measured_651_656.py --accepted-range-stress
simulation/check_parking_adaptive_completion.py --brake-overrun
simulation/check_parking_adaptive_completion.py
```

Die beiden Stressbefehle melden die verbleibenden Abbrüche als Fehler; sie
werden nicht stillschweigend zu erwarteten Erfolgen umdefiniert. Generierte
Ergebnisse liegen unter `local_workspace/`, insbesondere `lap1-reobserve/`,
`parking-adaptive-completion/`, `parking-adaptive-brake-overrun/` und
`returned-603/parking-handoff/` beziehungsweise `parking-current-capture-range/`.
Der Vergleich steht in `parking-adaptive-completion/comparison.json`.

## Build und nächste Abnahme

M7 IDE-PlatformIO: **SUCCESS**, RAM484896/523624B, Flash595792/786432B.
Binärdatei599344B, SHA-256:
`8aee74c3e54e96b9ace4bf3d7fbe941fa761ae9f4d0d4f21b0858402216fca53`.
Kennungen: `20261111_active_view_retry`, `20261111_recovery_handoff` und
`20261111_height_rule`. M4 unverändert und nicht gebaut. Keine Übertragung,
kein Commit oder Push. Vorhandene Serial-Warnung bleibt.

Nächster Nachweis sind normale CW-/CCW-Läufe mit der tatsächlichen Lücke und
unveränderter Hardware. Neue Logs zuerst unverändert kopieren, SHA-256 prüfen
und sofort Ausstecken freigeben. Dabei Suchfortsetzung, Wiederanschluss,
Einparkabschluss und tatsächliche Freiräume beobachten. Für die neue
Sonderregel fehlen weiterhin echte50/150-mm-Aufnahmen mit der montierten
Kamera; jedes zusätzliche Referenzbild vor Verwendung vom Nutzer bestätigen
lassen. Keine ungültigen oder abgelehnten Archivbilder verwenden.
