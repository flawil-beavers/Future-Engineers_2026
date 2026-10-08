# Robot Run Inspector

Eigenständiges Offline-Programm für Roboterlogs. Keine Installation, kein
Python, kein Server und keine Internetverbindung erforderlich.

## Start

`Start Robot Run Inspector.cmd` oder `Robot Run Inspector.html` doppelklicken.
In einem aktuellen Edge-, Chrome- oder Firefox-Browser öffnen. Den gesamten
Ordner auf den Wettbewerbslaptop kopieren; die HTML-Datei funktioniert auch
allein. Vor der Abreise bei ausgeschaltetem WLAN mit einem echten Log testen.

1. Originales TXT-Log vom USB-Stick auswählen oder ins Fenster ziehen.
2. Bei mehreren expliziten `[OC] New obstacle run`-Markern den Abschnitt wählen.
3. Befund, Sensorübersicht und Auffälligkeiten prüfen. Zeilenbuttons öffnen
   den Originalkontext. Logsuche und direkte Zeilenauswahl sind verfügbar.
4. Fahrspuren einzeln auswählen. Runden und Diagnosequellen bleiben getrennt.
5. Physische Beobachtungen und Video-Dateinamen ergänzen.
6. HTML-Bericht oder JSON speichern; Drucken ermöglicht die PDF-Ausgabe.

Der HTML-Bericht enthält den ausgewählten Abschnitt und alle seine Fahrspuren.
JSON enthält alle Abschnitte und die Originalzeilen. Downloads werden im vom
Browser gewählten Ordner gespeichert. Für Repository-Arbeit Berichte nach
`local_workspace/robot-run-inspector/` verschieben; Rohlogs separat unverändert
nach der Evidenz-Anleitung in `AGENTS.md` archivieren. Ein Bericht ist keine
Evidenzarchivierung. Beobachtungen bleiben bis zum Export nur im Arbeitsspeicher.

## Unterstützte Daten

- Build-Kennungen aus `build=` (Modul-Builds, kein Firmware-Binärnachweis).
- Explizite CW/CCW- und Completed-lap-Marker; keine Ableitung aus Dateinamen.
- Gyro-Sensor-Timeouts, Stream-Neustarts und getrennte Hauptschleifenpausen.
- Sicherheits-/Testhalte, Park-/Corner-Abbrüche, Park-Endprüfung, Schlupfindikatoren.
- Abgelaufene Hindernis-Beobachtungshalte, verworfene Routenanschlüsse und
  Anzahl/Maximalbetrag expliziter PARK_DIAG-Posekorrekturen.
- Logger-Überlauf und PARK_DIAG overflow/truncated-Marker.
- Positionen aus PARK_DIAG, CONNECTOR_TRACK, LATER_TRACK, DISCOVERY_TRACE und
  FINAL_PARK_TRACE. Gleicher Massstab beider Achsen; geschätzte Feldansicht, kein Kollisionsnachweis.
- Neue ToF-Datensätze aus PARK_DIAG v2 sowie seitliche FINAL_PARK_TRACE-Werte.
- SHA-256 der unveränderten importierten Dateibytes, wenn der Browser lokale
  Web-Crypto unterstützt. Sonst wird der fehlende Hash ausdrücklich angezeigt.

## Grenzen

Keine Roboterverbindung, kein Upload und keine zusätzlichen Fahrbewegungen.
Die bestehende Firmware und Diagnoseausgabe bleiben unverändert.
Keine Bildauswertung, Spannungsmessung oder automatische Ursachengarantie.
Ein Software-Containment-Ergebnis bestätigt nicht die tatsächliche Parkposition.
Fehlende Fehler-/Ergebnisdaten bedeuten unbekannt, nicht erfolgreich.
Ein vollständiges Original kann nicht allein aus dem Dateiinhalt bestätigt
werden. Dateien mit `excerpt` im Namen werden zusätzlich als Ausschnitt markiert.
Die Diagnoseaufzeichnung kann begrenzt sein, während das restliche Log weitergeht.
Bei zusammengefügten Logs ohne eindeutige Laufstartmarker ist keine sichere
Lauftrennung möglich; mehrere PARK_DIAG_CONFIG-Marker werden gemeldet.
Zeitangaben gelten nur für erfasste Datensätze, nicht als vollständige Laufdauer.
ToF-Statistiken sind Beobachtungswerte, keine Aussage über Hardwaregesundheit.
Dateien bis 20 MB; UTF-8/ASCII-Text, Binärdaten werden abgewiesen.

Alle Oberflächenressourcen sind eingebettet; die Content Security Policy
verbietet Netzwerkverbindungen. Dateiinhalt wird als Text behandelt.

## Entwicklerprüfung

Mit Node.js: `node tools/robot-run-inspector/test-inspector.cjs` aus dem
Repository-Root. Der Parser wird direkt aus der gelieferten HTML-Datei geprüft,
einschliesslich historischer Fehlerlogs und synthetischer Grenzfälle.
`node tools/robot-run-inspector/test-app.cjs` prüft Import, Abschnittswechsel,
Suche, Zeilensprung und Exporte mit einem minimalen DOM-Testadapter. Dies ist
keine visuelle Browserprüfung.

## Gesamtlauf mit Pfosten (Version 1.1)

Nach dem TXT-Import zeigt die neue Ansicht das gesamte Feld und ein Detail
für Ausparken, Anschluss und Endparken. Die bestehende Einzelspur-Auswahl bleibt
verfügbar. Farben trennen die Diagnosephasen und Runden; graue Punkte sind
vereinzelte Erkundungspositionen. Anschlusspläne sind gestrichelt, Scan-Endpunkte
und Posekorrekturen punktiert. Zeitlücken über 2 Sekunden, Zeitrücksprünge und
explizite Park-Posekorrekturen werden nicht als durchgehende Fahrt verbunden.
Die lokale hintere Positionierung vor dem Feldreset gehört nicht zur Feldkarte.

Pfosten erscheinen nur aus bestätigten Map-/Avoidance- oder gespeicherten
Startplatz-Records dieses Laufabschnitts. R/G plus Zahl bezeichnet Farbe und
Sitzindex. Zentren folgen nominalen CW/CCW-Sitzen (100 mm seitlicher Versatz,
500 mm Stationsabstand), nicht den verworfenen Kamera-Projektionen. Symbole
sind keine maßstäblichen Pfostenkörper; die Map kann unvollständig/falsch sein.
Ohne eindeutige Richtung werden Pfosten ausgelassen. Fehlende Fahrdaten bleiben
Lücken; eine volle Runde wird nicht aus einer Zeichnung behauptet.

`SVG speichern` exportiert die Ansicht des ausgewählten Abschnitts. Der normale
HTML-Bericht enthält sie ebenfalls. Alles wird direkt im Browser erzeugt;
Python, Matplotlib, Server und Internet sind weiterhin nicht erforderlich.

## Spielfeld-Ebene (Version 1.2)

`Spielfeld anzeigen` schaltet die schematische SVG-Unterlage ein/aus, ohne
Pfosten oder Telemetrie auszublenden. Innen-/Außenbereich, Eckabschnitte,
Abschnittsgrenzen, Stationszentren0/1/2, Parkbucht und Pfeile der protokollierten
CW/CCW-Richtung liegen im selben Millimeter-Koordinatensystem. Der Startabschnitt
ist im kanonischen Roboterrahmen immer unten (S0); die übrigen Abschnittsnummern
folgen der Fahrtrichtung. Bei unbekannter Richtung gibt es keine Richtungspfeile.
Stationskreise sind Orientierungspunkte, keine zusätzlichen Hindernisse.
SVG-Download und HTML-Bericht übernehmen die aktuelle Ebeneneinstellung.

Dies ist eine maßhaltige schematische Unterlage der verwendeten Projektgeometrie,
keine fotografische/identische Wiedergabe der bedruckten Matte und kein Beleg
für eine vermessene physische Feldplatzierung. Referenzen: `WRO_2026_RULES.md`,
[offizielle WRO-Regeln](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
(January15_2026) und [offizielle Q&A](https://wro-association.org/competition/questions-answers/),
am2026-10-08 erneut geprüft. Variable Pfosten bleiben eine getrennte Log-Ebene.
