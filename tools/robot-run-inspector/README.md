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
  FINAL_PARK_TRACE. Gleicher Massstab beider Achsen; keine Feld-/Kollisionskarte.
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
