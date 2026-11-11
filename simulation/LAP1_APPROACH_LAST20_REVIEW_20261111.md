# Runde 1: Blickrichtung und letzte 20 archivierte Dateien

Nachfolgende, separat gepruefte Firmwareaenderung:
[begrenzte CCW-Blickkorrektur](LAP1_CHECKED_CCW_VIEW_20261111.md).
Die folgende Auswertung beschreibt den davor untersuchten Stand.

Gepruefter Quellstand: `ad291e1`, Wegkennung `20261111_active_view_retry`.
Diese Untersuchung aendert **keine Firmware**, Geschwindigkeiten, Routen,
Sicherheitsabstaende oder Erkennungsschwellen. Kein Upload und keine Bewegung.

## Umfang und Beleggrenzen

Die letzten 20 vorhandenen Logdateien sind 629–636 und 645–656. 637–644 liegen
nicht im Archiv; sie wurden nicht als fehlerfreie Laeufe mitgezaehlt. Alle 20
Dateihashes wurden gegen den [Belegindex](evidence/parking_exit_diagnostics/README.md)
geprueft. 650 ist ein ausdruecklich gekennzeichneter Ausschnitt. 636 ist eine
vollstaendige Originaldatei ohne Laufabschluss. Kein neuer USB-Stick vorhanden.

652 wurde vom Benutzer unterbrochen; 653 hatte laut Benutzer rutschigen Boden.
630, 635/636, 649/650 erreichen keine auswertbare normale Einlesefahrt. Insgesamt
13 Dateien enthalten normale Runde-1-Fahr-/Erkennungsdiagnosen. Ein Lauf ohne
Suchereignis ist damit nicht automatisch physisch korrekt.

Die alten Laeufe verwenden Wegkennungen `20261031_far_green_rear30`,
`20261102_lap_handoff` oder `20261105_physical_lap_speeds`. Sie belegen nicht
das Verhalten der neuen Hoehenerkennung/Sonderregel oder der letzten Retrykorrektur.
Es wurden keine weiteren Kamerabilder als Referenz verwendet.

## Was die aktuelle Blicklogik tut

- Offene Stationen werden ab 850 mm Wegabstand betrachtet, mit voller
  Zielkorrektur ab 650 mm. Das sind Abstaende entlang des Grundwegs, keine
  garantierten Kameraentfernungen.
- Bei zwei offenen Reihen wird das Winkelmittel beider Plaetze verwendet.
  Der bevorzugte Winkelbereich betraegt etwa +/-24,4 Grad. Fuer Leerbelege
  gelten separat 230–600 mm Kameraentfernung und etwa +/-26,4 Grad.
- Ist nur eine Reihe offen, zielt die Korrektur staerker auf deren Richtung.
  Reale Sichtbarkeit und Zuordnung bleiben Voraussetzungen fuer Bestaetigungen.
- Eine bereits eingespeiste Umfahrung sperrt diese zusaetzliche Zielrotation
  bis 150 mm Wegfortschritt hinter dem betreffenden Block. Das schuetzt den
  zuvor geprueften Umfahrweg, kann aber das naechste Sichtfenster verkleinern.
- Ein passender GREEN-Randhinweis kann innerhalb 230–600 mm staerker nachfuehren;
  seine Kurve wird geprueft. Dies bestaetigt weder Farbe noch Belegung.
- Die 600-mm-Grenze fuer Leerbelege ist keine generelle Grenze fuer farbige
  Bloecke. Der getrennte Erkennungs-/Zuordnungsweg akzeptiert auch fernere
  brauchbare Beobachtungen.

## Neun Suchmanoevren in acht Dateien

Stationsnummern unten sind interne Indizes ab null. Distanzen sind die
geschaetzte Kameraentfernung beim abgelaufenen Wahrnehmungshalt, vor Ruecksetzen.
Die Zeitangabe endet bei Farb-/Leerbestaetigung, nicht erst bei Wiederanfahrt.

| Log | Ziel | Befund vor dem Ruecksetzen | Suchbewegung / Bestaetigung |
| --- | --- | --- | --- |
| 632 CW | Station 3, GREEN 7 nach Startkurve | Frueh -13,2 Grad bei 497 mm; danach aus dem Blick, spaet nur 200 mm entfernt. Rohkomponente mehrfach ungueltig. | 110 mm zurueck plus Parallaxe; GREEN nach 6,84 s bestaetigt. |
| 632 CW | Station 6, offener Platz 13 | Beim Halt 206 mm und damit unter Leerbeleg-Mindestdistanz. Keine fruehen Approach-Traces dieser Station gespeichert. | 60 mm zurueck; Platz nach 1,68 s als leer bestaetigt; weitere Rueckfahrt fuer sicheren Weganschluss. |
| 633 CCW | Station 9, RED 19 nach Kurve | RED-Platz +42,5 Grad bei 195 mm, also ausserhalb des brauchbaren Blickbereichs. | 130 mm zurueck plus Parallaxe; RED nach 7,26 s bestaetigt. |
| 645 CW | Station 3, GREEN 7 | Der belegte GREEN-Platz wurde zunaechst sogar als leer protokolliert und spaeter mit GREEN ueberschrieben. Zusaetzlich ungewoehnliche vorherige RED-Zuordnung zu Platz 2. | 140 mm zurueck plus Parallaxe; GREEN nach 12,80 s bestaetigt. Keine sichere Einzelursache ohne Originalbilder. |
| 648 CCW | Station 9, RED 19 | Platz nur 173 mm entfernt; roter Rohblob endet bei Bildzeile 238 und ist ungueltig. | 140 mm zurueck; RED nach 4,59 s bestaetigt. |
| 651 CW | Station 6, offener Platz 13 | Frueh -19,8 Grad bei 488 mm, danach aus dem Blick; spaet 202 mm. | 60 mm zurueck; Platz nach 1,63 s als leer bestaetigt; weitere Rueckfahrt fuer Weganschluss. |
| 654 CW | Station 3, GREEN 7 nach Startkurve | Frueh -3,7 Grad bei 509 mm, dann -34,2 Grad bei 439 mm. Am Halt 193 mm; breite/hohe Rohkomponente ungueltig. | Keine sichere gerade Rueckfahrt; gepruefte Parallaxe. GREEN nach 4,94 s bestaetigt. |
| 655 CCW | Station 9, RED 19 | Platz nur 164 mm entfernt; roter Rohblob erreicht Bildzeile 238, Hoehe 159 px, ungueltig. | 150 mm zurueck; RED nach 2,87 s bestaetigt. |
| 656 CCW | Station 9, RED 19 | Frueher Trace zeigt +67,9 Grad bei 552 mm; beim Halt noch 178 mm. Roter Rohblob endet bei Bildzeile 238. | 130 mm zurueck; RED nach 2,39 s bestaetigt. |

Alle neun protokollierten Suchmanoevren erreichen einen sicheren Weganschluss
laut Steuerungsprotokoll. Daraus folgt weder ein kontaktfreier physischer Lauf
noch eine generell fehlerfreie Karte. Die beiden CW-Station-6-Manoevren sind
**keine Bestaetigung eines uebersehenen mittleren GREEN**: Zunaechst fehlte der
Leerbeleg der davorliegenden Station. Das ist eine wichtige Unterscheidung.

## Abgleich mit aktuellem Code

`analyze_lap1_last20.py` extrahiert Halt, Sichtverlauf und Wiederaufnahme mit
Zeilennummern. `check_lap1_approach_view.py` fuehrt die aktuellen C++-Funktionen
fuer Kamerageometrie, Umfahrungsschutz und Zielkorrektur aus.

- **270** Kombinationen aus Richtung, Anfahrdistanz, Seitenversatz, Winkel und
  offenen Reihen: korrekte Richtung der fruehen Zielkorrektur.
- **365/365** archivierte Diagnosepositionen: beide Kamerawinkel innerhalb
  0,15 Grad der gespeicherten Angaben reproduziert.
- Bei den fruehen CW-Traces aus 632/Station 3, 651/Station 6 und 654/Station 3
  greift der aktuelle Umfahrungsschutz weiterhin. Jeweils vier gespeicherte
  Approach-Positionen liegen darin; dort war die alte Zielkorrektur null.
- **52** Fernzuordnungsfaelle bei 646/700 mm erfolgreich. Dieser Test schaltet
  die Sonderregel gezielt aus und isoliert den normalen Zuordnungsweg.
- Sonderregel separat: **648** gerenderte, **48** Zuordnungs- und **24**
  Richtlinienfaelle erfolgreich, plus die eine zuvor freigegebene reale
  Normal-GREEN-Referenz. Das beweist keine reale 50/150-mm-Fernleistung.

Die Wiederholung an alten Positionen ist **keine neue dynamische Fahrt**.
Sie simuliert weder alte Bildinhalte noch Schlupf, Lenkverzoegerung oder eine
durch Aenderungen entstandene neue Fahrzeugbahn.

## Bewertung und konkrete naechste Arbeit

Eine generelle vertauschte Blickrichtung wurde nicht gefunden. Frueherkennung
ist dennoch nicht ueberall optimal: Bei CW geht ein kurzes Sichtfenster nach
der vorherigen Umfahrung verloren; bei CCW erfolgt der brauchbare Blick auf
RED 19 wiederholt erst zu nah. Form-/Tiefen-/Reihenzuordnungsprobleme tragen
zusaetzlich bei. Ein gueltiger Winkel allein bestaetigt keinen Block.

Ein einfaches Entfernen der 150-mm-Sperre waere keine belegte sichere Loesung.
Als gezielte Folgemassnahme sollte ein frueherer Blick nach **tatsaechlich
geprueftem vollstaendigem Freifahren des vorherigen Blocks** gerechnet werden,
einschliesslich quantisiertem CAD-Lenkwinkel, ganzem Fahrzeug und richtiger
Passierseite. Erst nach dynamischem Vergleich dieser Situationen und bislang
guten Anfahrten sollte daraus eine Firmwareaenderung werden. Leerschwellen
oder Farbzuordnung pauschal zu lockern ist durch diese Logs nicht begruendet.

Reproduzierbare Berichte liegen unter `local_workspace/lap1-last20/`; sie
enthalten Dateinamen, SHA-256, Zeilennummern und Diagnosewerte. Die Originale
und ihr Belegindex bleiben massgebend. Kein Firmwarebuild erforderlich, da
ausschliesslich Auswertung, Hostpruefungen und Dokumentation ergaenzt wurden.
