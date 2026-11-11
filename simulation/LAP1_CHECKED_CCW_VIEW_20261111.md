# Begrenzte CCW-Blickkorrektur in Runde 1

Folgeaenderung zur [Auswertung der letzten20 Dateien](LAP1_APPROACH_LAST20_REVIEW_20261111.md).
Nur `src/obstacle_path.cpp` ist als Firmwaredatei betroffen. Kennung:
`20261111_checked_ccw_view`. Kein Upload oder physischer Test durch den Agenten.

## Verhalten und Grenzen

Der vorhandene Umfahrungsschutz bleibt unveraendert: keine zusaetzliche
Blickrotation waehrend der geschuetzten Umfahrung, Freigabe weiterhin150mm
nach dem Block entlang des Grundwegs. Ein zunaechst untersuchter frueherer
Ausstieg nach gemessener Fahrzeugfreigabe wurde NICHT uebernommen; seine
dynamischen Gegenproben verschlechterten den spaeteren Blickwinkel.

Die uebernommene Aenderung betrifft nur CCW/Runde1, nach dieser Freigabe:

- Nur ein Blockplatz der ausgewaehlten Station ist noch offen.
- Die normale Blickkorrektur ist bereits nahezu am45-Grad-Zielrotationslimit.
- Der offene Platz liegt noch ausserhalb des etwa26,4-Grad-Blickfensters,
  bei300–750mm geschaetzter Kameraentfernung.
- Dann darf die Zielrotation mit dem vorhandenen60Grad/s-Aufbau bis50Grad
  wachsen. Dies ist ein Pure-Pursuit-Zielwinkel, kein neuer maximaler
  Lenkeinschlag; dessen42Grad-Grenze bleibt bestehen.
- Vor jeder solchen Ausgabe werden80mm des tatsaechlich quantisierten
  CAD-Lenkbogens in28Positionen geprueft. Der bestehende40mm-Abstand zu Waenden
  und allen noch moeglichen Blockpositionen bleibt erhalten. Die Huellkontur
  schliesst den gemessenen135mm-Frontueberstand vom Hinterachsmittelpunkt ein,
  ohne die bisherige Heckreserve zu entfernen.
- Auch beim weichen Rueckgang einer zuvor erlaubten Zusatzrotation wird sie
  erneut geprueft. Bei Ablehnung gilt wieder der normale Zielwinkelbereich.
  Es entstehen kein neuer Haltzustand, keine Scanpause und keine Fahrfreigabe.

Geschwindigkeiten, Farbschwellen, Hoehenregel, Leerbestaetigungen, normale
Zwei-Platz-Suche, Ausparken, Parken und aufgezeichnete Routen bleiben erhalten.
CW erhaelt keine zusaetzliche Zielrotation.

## Gegenproben

`check_early_lap1_aim.py` vergleicht aktuelle C++-Steuerung und vorherigen
Umfahrungsschutz unter denselben synthetischen Kamera-/CAD-Bedingungen.
Sieben Ausgangssituationen, jeweils81Varianten mit+/-5mm X/Y,+/-1Grad und
Kurvenwirkung0,9/1,0/1,1: **567 Anfahrten** bis zum bestehenden Wahrnehmungshalt.

Beim Problemfall CCW655/RED19 gewinnt **75/81** ein zusammenhaengendes
geometrisches Sichtfenster von80–320ms. **60/81** erreichen mindestens160ms;
sechsVarianten bleiben ohne Sichtfenster. Im nominellen Fall entstehen240ms
bei zunaechst291mm Kameraentfernung. Der spaete Winkel am Haltepunkt ist dabei
nicht besser (maximal0,574Grad schlechter); der Vorteil ist das fruehere Fenster.
Die uebrigen486Varianten, darunter alle324CW-Faelle und zwei weitere
CCW-Anfahrten, sind in den aufgezeichneten Modellmetriken identisch zur
Vorversion. Kein bestehendes Zwei-Frame-Sichtfenster geht verloren.

**362** gezielte Dispatch-/Rueckgangsfaelle pruefen Richtung, Runde, offene
Reihen, Distanz, Winkel und erlaubte/abgelehnte Bewegung. Negative Kontrolle
ohne den neuen Preflight scheitert erwartungsgemaess. Der reale
Umfahrungsschutz und die Wandablehnung werden separat direkt ausgefuehrt.

Zusaetzlich bestanden270Blickrichtungsfaelle,365archivierte Kamerageometrien
und1736bestehende Known-Map-Runde1-Geometriefaelle. Letztere modellieren keine
Wahrnehmung. Erkennungs-/Hoehenlogik wurde nicht geaendert.

Dies ist **keine reale Erkennungsquote**. Bilder, Farb-/Hoehenbestaetigung,
Regelkreisverzoegerung, mechanische Toleranzen und Schlupf werden nicht
vollstaendig nachgebildet. Ein80ms-Fenster reicht auch im Modell nicht fuer
zwei Bilder. Bestehende Suchmanoevren bleiben deshalb erforderlich. Der
bekannte schmale232,5mm-Parkspalt ist durch diese Aenderung nicht behoben.

Ergebnisse/negative Kontrollen: `local_workspace/early-lap1-aim/`.
Originalbelege bleiben unveraendert im verlinkten Archiv. Nur M7 wurde mit der
IDE-verwalteten PlatformIO-Installation gebaut; M4 bleibt unveraendert.
BuildSUCCESS, RAM484896/523624B, Flash597064/786432B, Binary600616B.
SHA-256: `fb49725aadef397cbcce99110e5239eb5fd919883c496da24b275de948ba3765`.
