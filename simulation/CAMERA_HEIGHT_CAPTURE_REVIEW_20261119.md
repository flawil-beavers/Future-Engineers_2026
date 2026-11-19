# Stationaere Hoehenpruefung vom 19. November 2026

48 vollstaendige Aufnahmen aus 16 Aufstellungen; 45 vom Nutzer freigegeben, drei ausgeschlossen. Abstaende: Linse bis nahe Blockflaeche. Keine Fahrbewegung, Firmwareaenderung oder Uebertragung. Alle 192 Bild-/Roh-/Metadatendateien gegen SHA-256 und Git-Index geprueft; alle 48 seriellen Nutzdaten CRC/Footer-geprueft und mit RGB565 verglichen. Freigaben und Einzeldateihashes: [Registry](evidence/camera_diagnostics/validation_status.json), [Originale](evidence/camera_diagnostics/README.md).

## Befund

Die bestehende Software besteht die neuen realen Referenzen nicht. Nur 11 von 45 freigegebenen Bildern liefern fuer die erwartete Farbe die richtige Hoehenklasse. In sieben Bildern fehlt sogar eine Komponente der erwarteten Farbe. Eine gefundene Komponente beweist noch keine korrekte Zielzuordnung: GREEN50/700 zeigt im dritten Bild eine abseits liegende Hintergrundkomponente. Ein weiteres Hintergrund-GREEN erscheint in einem RED150/350-Bild.

Die gemeldete Kameravorwaertsdistanz liegt bei klaren RED-Referenzen ungefaehr bei317 statt350mm,382 statt500mm und454 statt700mm. Das betrifft beide Farben und alle Hoehen mit gleichem Bodenpunkt. Es ist daher kein ausreichender Ansatz, nur Hoehenfenster aufzuweiten: Die Boden-/Entfernungsgeometrie muss gegen die aktuelle Montage geprueft werden. Insbesondere wird150mm bei500mm derzeit als100mm klassifiziert; die Sonderaufgabe ist so nicht physisch abgenommen.

| Farbe | Sollhoehe mm | Sollabstand mm | Freigegeben | Komponente erwarteter Farbe | Richtige Hoehenklasse | Modellhoehe mm | Modelldistanz mm |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| GREEN | 50 | 350 | 2 | 2 | 2 | 48.4205 | 316.981 |
| GREEN | 50 | 500 | 2 | 2 | 2 | 37.4483, 39.647 | 381.818, 390.698 |
| GREEN | 50 | 700 | 3 | 1 | 0 | 24.3748 | 622.222 |
| GREEN | 100 | 500 | 2 | 2 | 0 | 74.3126 | 381.818 |
| GREEN | 100 | 700 | 3 | 3 | 0 | 60.2199 | 454.054 |
| GREEN | 150 | 350 | 3 | 3 | 0 | 129.963 | 316.981 |
| GREEN | 150 | 500 | 3 | 2 | 0 | 112.599, 117.69 | 381.818, 390.698 |
| GREEN | 150 | 700 | 3 | 2 | 0 | 93.6617 | 454.054 |
| RED | 50 | 350 | 3 | 3 | 3 | 48.4205 | 316.981 |
| RED | 50 | 500 | 3 | 3 | 3 | 39.647 | 381.818 |
| RED | 50 | 700 | 3 | 0 | 0 | - | - |
| RED | 100 | 500 | 3 | 3 | 0 | 74.3126 | 381.818 |
| RED | 100 | 700 | 3 | 3 | 0 | 60.2199 | 454.054 |
| RED | 150 | 350 | 3 | 3 | 1 | 129.963, 132.059 | 316.981 |
| RED | 150 | 500 | 3 | 3 | 0 | 112.599 | 381.818 |
| RED | 150 | 700 | 3 | 3 | 0 | 93.6617 | 454.054 |

## Methode und Grenzen

Replay der tatsaechlichen Vision-/Hoehenimplementierung im Host-Fixture; 648 synthetische Bilder,48 Zuordnungsfaelle und24 Regel-/Fehlerfaelle bestehen weiterhin. Das ersetzt keinen physischen Nachweis. Der Replay prueft die aktuelle Arbeitskopie, nicht die unbekannte Firmwareidentitaet des angeschlossenen Roboters. Das Fixture ersetzt Teile der nachfolgenden Kartenmutation. Eine Einzelbildklassifikation prueft keine zeitliche Dreiframe-Bestaetigung waehrend Fahrt. Alle Ziele standen mittig; seitliche Anfahrt, Vibration und wechselndes Licht sind nicht abgedeckt. Farbgrenzen wurden nicht veraendert.

`check_surprise_rule.py` nutzt jetzt bei neuen Referenzen deren explizite Sollhoehe/Farbe statt alle freigegebenen Bilder als100mm anzunehmen. Der Test meldet bewusst Fehler; die beobachteten Abweichungen werden nicht zu bestandenen Tests umdefiniert. Reproduzierbare Detailausgaben liegen unter `local_workspace/surprise-rule/`.

## Naechste Schritte

1. Bodenpunkt-/Entfernungsmodell und Hoehenprojektion gemeinsam aus den freigegebenen gemessenen Aufstellungen pruefen; Modell nicht durch blosse Klassenaufweitung kaschieren.
2. Danach die kleinen70cm-Pfosten und die mit dem Hintergrund verbundenen GREEN-Komponenten gezielt offline pruefen, ohne ungepruefte Farbgrenzen zu lockern.
3. Alle freigegebenen Aufstellungen inklusive normal100mm als Regression erhalten; historische Referenzen als andere Montagesitzung behandeln.
4. Erst nach belastbarem Offline-Ergebnis gezielte M7-Aenderung bauen; anschliessend normale CW/CCW-Laeufe zur realen Abnahme.

Replay-Quellidentitaet (SHA-256):

- `src/vision.cpp`: `c0d75193b0257de2aaca09ecc1218c08d27bd54f6c6abffe29290fc674f00f99`
- `include/config.h`: `85eaa23f5ffd63754fbf4d404be8343291d8ad7764d82fe27b646db9b6c5585b`
- `include/obstacle_sign_height.h`: `e81121e081f852ee4baa0a1ccc7cc6f8686f02524b076846020fe9b0b71062f5`
- `src/obstacle_path.cpp`: `47ff112708d42de496dbfb52cc810890a1080ca9a1ba73ceb5a92995abae644d`
- `simulation/check_surprise_rule.py`: `9149ae62c5da8ff8a7879b56cec0f8fb8aaecf09d7d543a254048bc682f21b0c`
