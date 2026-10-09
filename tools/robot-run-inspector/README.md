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

## Offizielle FE-Spielmatte (Version 1.4)

Der Hintergrund verwendet die offizielle Spielmatte, einschließlich der
blauen/orangen Linien, gedruckten Startfelder, Pfostensitze und Mittelgrafik.
Die frühere schematische Ebene, Stationsbeschriftungen, Richtungspfeile und
`Spielfeld anzeigen` wurden entfernt. Der Hintergrund ist immer sichtbar,
auch ohne Feldpositionsdaten. Lokale Positionen bleiben in der Einzelspur.

Quelle: [WRO 2026 FE Game Mat](https://wro-association.org/wp-content/uploads/WRO-2026_FutureEngineers_Playfield.pdf),
verlinkt auf der [offiziellen Saisonseite](https://wro-association.org/competition/2026-season/).
Saisonseite und [Q&A](https://wro-association.org/competition/questions-answers/)
am 2026-10-09 geprüft. Artwork und Marken gehören ihren jeweiligen Eigentümern;
die Umwandlung macht daraus kein eigenes Matten-Design.

`assets/fe-2026-mat.svg` ist aus der offiziellen PDF mit `pdftocairo -svg`
abgeleitet; `assets/fe-2026-mat.json` enthält Quellen-/Asset-Hashes und PDF-Boxen.
Die MediaBox umfasst 3210 mm einschließlich Beschnitt. Die TrimBox schneidet
5 mm pro Seite ab und ergibt 3200 × 3200 mm (PDF-Rundungsabweichung <0,01 mm).
Die vollständige TrimBox liegt bei X/Y −1600…+1600 mm, Mittelpunkt (0,0),
+X nach rechts und +Y nach oben. Außenwände liegen nominal bei ±1500 mm,
Innenwände bei ±500 mm. Der Startabschnitt ist unten; CW/CCW spiegelt die
Matte nicht. Beide Ansichten nutzen denselben Maßstab/Transform wie die Spuren.
Die gedruckten 24 Pfostensitze stimmen innerhalb 0,6 mm mit den nominalen
Roboterkoordinaten überein; keine unabhängige Vermessung des echten Tisches.

Wände sind eine getrennte nominale Overlay-Ebene. Magenta-Parkbegrenzungen
werden nur bei protokolliertem `fixed_line_x` und `gap_mm` gezeichnet; fehlende
Geometrie wird als unbekannt bezeichnet. Gedruckte Pfostensitze bedeuten keine
vorhandenen Pfosten. Nur bestätigte Log-Sitze erscheinen als farbige Objekte.
Spuren/Pläne liegen darüber und haben einen dünnen weißen Kontrastrand.

Das SVG ist in die HTML eingebettet und wird in beiden Ansichten wiederverwendet.
SVG-Download und HTML-Bericht enthalten die Grafik ebenfalls; keine externen
Dateien/Fonts/Netzwerkzugriffe zur Laufzeit. Wegen des vollständigen Artworks
ist die HTML etwa 8,6 MB groß. Planwahl, Zeitdaten und bestehende Log-Trennung
bleiben erhalten.

Prüfen (ohne Zusatzbibliotheken):

```text
python tools/robot-run-inspector/check-field-asset.py
node tools/robot-run-inspector/test-inspector.cjs
node tools/robot-run-inspector/test-app.cjs
```

Nur zur Regeneration sind Python und Popplers `pdftocairo` auf PATH nötig:

```text
python tools/robot-run-inspector/build-field-asset.py
```

Das Programm lädt eine fehlende Quellen-PDF nach `local_workspace/`, überprüft
SHA-256, erzeugt SVG/Metadaten und aktualisiert die markierte Einbettung in der
HTML. Bei geänderter Quelle stoppt es zur Prüfung von Beschnitt/Ausrichtung.
Die PDF und rohe Konvertierung bleiben ignoriert. SVG-Bytes können zwischen
Poppler-Versionen variieren; Asset-Metadaten und HTML werden zusammen erzeugt.

## Lauftelemetrie und Zeit (Version 1.3)

`RUN_POSE` v1 zeichnet die geschätzte Spur über den ganzen Lauf. Lokale
Positionen sind in der Einzelspur verfügbar, Feldpositionen im Gesamtlauf.
Fortlaufende Messpunkte verbinden auch Anschluss und Runde; Korrekturen,
Rahmenwechsel und Zeitlücken bleiben getrennt. Der Planselektor zeigt die zuletzt
aktive Route, eine gewählte protokollierte Version in der gewählten Runde.
Gestrichelte Linien sind akzeptierte Pläne, keine gefahrenen Positionen. Fehlende
Basisversionen und unvollständige Routenblöcke werden nicht gezeichnet.

Gesamtzeit und Rundenzeiten kommen aus `RUN_START`, `RUN_LAP` und `RUN_END`.
Halte, Sensorpausen, Bremsen und abschließendes Parken zählen zur Gesamtzeit.
Ohne Start und bestätigten Abschluss, bei Abbruch/Stopp oder Excerpt bleibt
die Abschlusszeit unbekannt; die beobachtete Zeitspanne wird separat angezeigt.
Alte Logs erhalten keine nachträglich erfundene erfolgreiche Abschlusszeit.
Begrenzte Telemetrie wird sichtbar gemeldet. SVG/HTML-Exporte übernehmen die
Planwahl, HTML enthält auch die Zeiten. Details und normale CW/CCW-Prüfung:
[`simulation/RUN_TELEMETRY.md`](../../simulation/RUN_TELEMETRY.md).

### Darstellung von Posekorrekturen

Violett punktiert verbindet ausschließlich die protokollierten Vorher-/Nachher-
X/Y-Positionen einer Korrektur im Feldrahmen, nicht eine gefahrene Strecke.
Die Legende verwendet dasselbe Punktmuster; der SVG-Titel nennt die Endpunkte.
Lokale/unbekannte Korrekturrahmen und Rebases werden nicht als Feldsprung
gezeichnet. Explizite Korrekturen unterbrechen die durchgezogene Fahrspur.
Bei gemischten Formaten bleiben PARK_DIAG-Korrekturen erhalten, wenn neue
RUN_EVENT-Daten fehlen; doppelte Meldungen desselben Sprungs werden einmal
gezeichnet (Zeitabstand maximal10ms, X/Y-Rundungsabweichung maximal0,15mm).
Kleine Korrekturen sind im Gesamtlauf entsprechend kurz; z.B. log476:27,1mm.


## Runden- und Spurauswahl (Version 1.5)

Im Gesamtlauf wählt „Runde“ alle Runden, Runde 1/2/3 oder „Parken / ohne Runde“.
Beide Feldansichten und SVG-/HTML-Exporte übernehmen diese Auswahl. Feld und
bestätigte Pfosten bleiben sichtbar. Aus- und Endparken gehören zur separaten
Parkauswahl, auch wenn ein Parkdatensatz noch die Nummer der letzten Runde trägt.
Der Anschluss gehört zur Runde 1. Versteckte Phasen werden nicht durch eine
neue Verbindung überbrückt. Posekorrekturen folgen ihrer protokollierten Phase.

„FE-Spielfeld“ zeigt oder versteckt ausschließlich das offizielle Matten-Artwork.
Wände, bestätigte Pfosten und bekannte Parkbegrenzungen bleiben in beiden
Ansichten sichtbar, ebenso Fahrspur und ausgewählter Plan. SVG-/HTML-Exporte
übernehmen die Hintergrundwahl. Die separate Auswahl „Spuren“ zeigt Plan + Fahrspur, nur Fahrspur oder nur
Plan, unabhängig vom Hintergrund. Der Planversionsselektor mit „Letzter Plan
in Auswahl“ erscheint nur bei mehreren Läufen/Abschnitten in derselben Datei;
bei einer einzelnen Sitzung wird dieser Plan automatisch gewählt. Geschätzte Fahrt ist
dick, farbig und durchgezogen; akzeptierte Pläne sind dünn, dunkel gestrichelt
und liegen darüber. Violett punktiert bleibt ausschließlich Posekorrektur.
„Letzter Plan in Auswahl“ verwendet die Route des letzten Feldpositionsdatensatzes
der gewählten Runde. Eine explizite Planversion erlaubt den manuellen Vergleich
mit einer anderen protokollierten Version. Es ist eine ausgewählte Planversion,
kein zeitlicher Replay aller während einer Runde aktivierten Versionen.

Der Filter erleichtert Plan-/Ist-Vergleiche ohne überlagerte Runden. „Alle Runden“
bleibt für Übergänge und den gesamten Ablauf sinnvoll. Übersicht, Zeiten, Rohlog
und separate Einzelspur-Auswahl bleiben vom Gesamtlauf-Filter unabhängig.
Alte Logs zeigen nur vorhandene Daten: einzelne Erkundungspositionen in Runde 1,
spätere LATER_TRACK-Punkte in ihrer Runde, Anschlusspläne nur in Runde 1.
Fehlende Fahrspuren oder Pläne werden ausdrücklich gemeldet, nicht ergänzt.
