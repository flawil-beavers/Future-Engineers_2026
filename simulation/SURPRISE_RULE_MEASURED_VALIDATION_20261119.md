# Gemessene Sonderregel-Version vom 19. November 2026

## Ergebnis und Aenderung

Alle45 vom Nutzer freigegebenen Bilder aus der aktuellen Montage liefern jetzt die richtige sichtbare Farbe und Hoehenklasse50/100/150mm. Keine fehlende Zielkomponente, kein Fehler ueber30mm in der gemessenen Kameravorwaertsdistanz; groesster gemessener Fehler13.1mm. Die drei ausgeschlossenen Bilder bleiben ausgeschlossen. Die45 Bilder decken mittige stationaere Ziele350/500/700mm ab;100mm wurde nur500/700mm aufgenommen. Dies ist ein Offline-Nachweis an realen Bildern, keine Zuverlaessigkeitsgarantie waehrend Fahrt oder bei anderen Blickwinkeln.

Ausgangsbefund mit alter Kalibrierung: nur11/45 richtige Klassen, siehe [unveraenderter Ausgangsbericht](CAMERA_HEIGHT_CAPTURE_REVIEW_20261119.md). Die urspruenglichen Bilder und Freigaben bleiben unter [Kameraevidenz](evidence/camera_diagnostics/README.md). Alle48 kompletten Rohbildtransfers wurden CRC/Footer-geprueft;192 Dateien stimmen mit SHA256 und Git-Index ueberein.

- Bodenmodell der aktuellen Montage: Horizont94px, Skala22400mm*px; vertikale Brennweite259px. Gemessene abgetastete Bodenpunkte158/140/126px bei350/500/700mm. Eine gleich gewichtete1/Distanz-Ausgleichsrechnung ueber diese drei Stationen ergibt ungefaehr94.6px/22430mm*px. Die gerundete Wahl bleibt innerhalb der gemessenen Fehler. Der gemeinsame vertikale Pinhole-Abgleich der roten50/100/150mm-Aufstellungen liefert ungefaehr259px; GREEN bleibt ein unabhaengiger Farbvergleich. Modellfenster fuer Hoehen und bestehende HSV-Grenzen wurden nicht erweitert.
- An zwei GREEN150-Aufnahmen war der Block mit einem gleichfarbigen Hintergrundstreifen verbunden. Die bereits gemessene untere Stammbreite/-mitte begrenzt jetzt die Kandidatengeometrie; Pixel werden mit unveraenderter HSV-Klassifikation nachgezaehlt. Die volle rohe Komponente bleibt fuer Belegung und Diagnose erhalten. Raumweite Komponenten, fehlende Hoehenmessung und duenne Linien erhalten dadurch keinen Freipass.
- `rememberModifiedSign()` speichert die eine Sonderposition samt Hoehe. `visibleRed` bleibt die sichtbare Farbe; `seat.red` ist die urspruengliche wirksame Farbe fuer Karte, Umfahrung und spaetere Runden. Der bestehende Pfad wird erst nach dieser Festlegung aufgebaut. Erste tatsaechlich angetroffene Saeule bleibt geschuetzt; ein spaeterer Block in der Startgeraden ist laut Regel moeglich. Kein zweiter Sonderblock und kein erneutes Umdrehen der Farbe.
- Drei frische Frames fuer einen Sonderblock, normale Zwei-Frame-Bestaetigung, bestehendes Ablauf-/Widerspruchsverhalten und Kartenfreeze in Runde2/3 bleiben bestehen. Zwei freigegebene GREEN50-Bildfolgen enthalten nur zwei Bilder: diese bestaetigen korrekterweise noch keinen Sonderblock.

## Aktuelle Bildmessungen

| Farbe | Hoehe mm | Abstand mm | Bilder korrekt | Modellhoehe mm | Modelldistanz mm |
| --- | ---: | ---: | ---: | --- | --- |
| GREEN | 50 | 350 | 2 | 51.4..51.4 | 350.0..350.0 |
| GREEN | 50 | 500 | 2 | 47.1..48.8 | 487.0..509.1 |
| GREEN | 50 | 700 | 3 | 59.1..59.1 | 700.0..700.0 |
| GREEN | 100 | 500 | 2 | 93.0..93.0 | 487.0..487.0 |
| GREEN | 100 | 700 | 3 | 90.9..90.9 | 700.0..700.0 |
| GREEN | 150 | 350 | 3 | 144.7..144.7 | 350.0..350.0 |
| GREEN | 150 | 500 | 3 | 143.5..153.7 | 487.0..509.1 |
| GREEN | 150 | 700 | 3 | 143.4..143.4 | 700.0..700.0 |
| RED | 50 | 350 | 3 | 51.4..51.4 | 350.0..350.0 |
| RED | 50 | 500 | 3 | 48.8..48.8 | 487.0..487.0 |
| RED | 50 | 700 | 3 | 53.8..53.8 | 700.0..700.0 |
| RED | 100 | 500 | 3 | 93.0..93.0 | 487.0..487.0 |
| RED | 100 | 700 | 3 | 90.9..90.9 | 700.0..700.0 |
| RED | 150 | 350 | 3 | 144.7..147.2 | 350.0..350.0 |
| RED | 150 | 500 | 3 | 143.5..143.5 | 487.0..487.0 |
| RED | 150 | 700 | 3 | 143.4..143.4 | 700.0..700.0 |

## Pruefungen

- `simulation/check_surprise_rule.py`:648 synthetische Projektionen,48 Zuordnungen CW/CCW,24 Regel-/Fehlerfaelle;45 hashgepruefte aktuelle reale Bilder mit Hoehe/Farbe/Abstand/Zielposition;16 echte Bildfolgen pruefen Bestaetigung und gespeicherte Originalfarbe bis Runde3. Alle bestanden. Reale Zweiframe-Sonderfolgen bleiben erwartungsgemaess unbestaetigt.
- `simulation/check_far_obstacle_acquisition.py`:52 Faelle bestanden, inklusive Kartenfreeze und Umfahrungsdispatch.
- `simulation/check_green_seat_visibility.py`: synthetische Belegung/Helligkeit/Hintergrund-/Bodenlinien, Fussgate und Negativkontrolle bestanden. Sein fester Archiv-Abfragepunkt ist kein Nachweis fuer alle Sonderhoehen.
- `simulation/check_parking_seat_foot.py`: historische Pixel unter ihrer Oktober-Kalibrierung plus aktuelle geometrische Fussgates bestanden. Historische Pixelkoordinaten wurden nicht faelschlich als neue Montage behandelt.
- `simulation/check_later_laps.py`:2807 Routen-/Zustandsfaelle bestanden.
- `simulation/check_lap_handoff.py`: alle sechs Richtungs-/Geschwindigkeitsfaelle bestanden, kein Zielpunktsprung beim Uebergang.
- `scripts/test-camera-image.py`5 und `scripts/test-camera-pillar-support.py`6 Tests bestanden.

IDE-PlatformIO `giga_r1_m7`: SUCCESS; RAM484896/523624B, Flash597432/786432B; Firmware600984B, SHA256 `059f6fb03f35efc3f11c10cfcb8c17e32dd08d22234a14d4dfacdcef51bfc92a`. M4 wird von diesen Aenderungen nicht verwendet und wurde nicht gebaut. Keine Uebertragung auf den Roboter. Bestehende Serial-/Legacy-Warnungen bleiben.

## Zusammenfuehrung und naechster physischer Nachweis

Vier Remote-Commits bis `caa2709` heruntergeladen und per Fast-forward zusammengefuehrt. Sie betreffen Visualisierung und Dokumentationshistorie. Den einzigen Konflikt beim Wiederanwenden lokaler Aenderungen im Handoff durch Beibehalten der neuen knappen Remote-Struktur und Archivierung des lokalen Originaltexts geloest. Keine Remote-Fahrcodeaenderung ging verloren. Lokale Mess-/Implementierungsaenderungen werden darueber einmalig committed; kein Push beauftragt.

Routen, Geschwindigkeit, Lenkparameter und Einparksteuerung wurden nicht angepasst. Die neue Kamerageometrie wirkt allerdings auf alle Kameradistanz-/Platzprojektionen; daher muss sie mit unveraenderter jetziger Montage bei normalen CW-/CCW-Fahrten geprueft werden. Teilnahme vor dem Start passend zur weiterhin aktivierten Sonderregel deklarieren. M7 normal selbst hochladen; erster normaler Block und jeweils ein50/150mm-Sonderblock pro Lauf, erwartete Originalfarbe und korrekte Seite auf allen drei Runden beobachten. Neue komplette Logs zuerst sichern; darin `[SURPRISE] revision=20261119_measured_height` und genau eine `locked seat/height/visible/original`-Zeile pruefen.

Bekannte separate Park-Limitation232.5mm bleibt im aktiven Handoff dokumentiert. Kein stehender Kameratest beweist vollstaendiges Parken oder fehlerfreie dynamische Erkennung. Keine weiteren Sonderbewegungen fuer Diagnose erforderlich.

Regeln am19. November erneut ueber die [offizielle Saisonseite](https://wro-association.org/competition/2026-season/), die [Sonderregel vom9. November](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf) und [offizielle Q&A](https://wro-association.org/competition/questions-answers/) geprueft; keine neue Sonderregel-Klarstellung gefunden.

Quellidentitaet (SHA256):

- `src/vision.cpp`: `89615bf7c65450de85e27e3d2c413c0116a3f75894cd6c374f3d4384be04d7cb`
- `src/obstacle_path.cpp`: `39476fa7543f6a6e4c4c34e3649a4eea09a13b58a3826979a79f0923bac2778f`
- `include/config.h`: `9bd289ba8a01aec77b0f2186bd6b1ac002b013087ac719a4080174378dfd71fb`
- `include/vision.h`: `c112c3ee7596627cf4d6f4921f8f0dc878572b1b278568ed00503d9f25b5f180`
- `include/obstacle_sign_height.h`: `e81121e081f852ee4baa0a1ccc7cc6f8686f02524b076846020fe9b0b71062f5`
