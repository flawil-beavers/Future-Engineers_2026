# USB-Speicherabsturz: Heap-Pruefung und Diagnose, 2026-11-19

## Gesicherter Befund

Der Nutzer meldet vier kurze/vier lange rote Blinkzeichen erst beim Speichern. Das entspricht dem dokumentierten MbedOS-Absturz; ein abgeschlossener Speichervorgang ist nicht nachgewiesen. Die656 vorherigen Stickdateien waren unveraendert, keine neue Sonderregel-Revision vorhanden. Der Nutzer hat den Stick anschliessend selbst geleert. Der gleiche16GB-FAT32-Stick besteht Windows-Dateisystempruefung und32 Schreib-/Lesetests mit Flush/Close/SHA256 (8MiB), ohne Fehler. [Portable PC-Pruefung](evidence/log_intake/20261119_save_check/README.md). Das prueft nicht die USB-Stromversorgung oder Kompatibilitaet am Roboter.

Im abgestuerzten Zustand erschien weder USB-CDC noch DFU; kein RAM-Dump moeglich. Keine Reset-/Upload-/Fahraktion durch den Agenten. Die genaue installierte Firmware, der Mbed-Fehlercode und der ausloesende Stackframe sind unbekannt. Ein Speichermangel ist ein konkreter Risikofaktor, aber noch KEINE bewiesene Absturzursache.

## Konkretes RAM-Risiko

Vorheriges ELF: __end__=0x24076630, __HeapLimit=0x2407fc00, damit38352B gesamte Heap-Spanne vor Laufzeitbelegung/Fragmentierung. Statisches RAM484896/523624B. Erst bei msd.connect() erzeugt USBHost::getHostInst() dynamisch einen3644B Host (aus dem ARM-Code nachgewiesen), plus3072B USB-Threadstack und weitere Verwaltungs-/Dateisystemobjekte. Ein erfolgreicher Link beweist daher nicht, dass zur spaeteren Speicherzeit alle Allokationen gelingen. Der32KiB-Hauptthreadstack ist bereits in .bss und wird hierbei nicht nochmals als Heapverbrauch gerechnet.

## Gezielt vorbereitete Aenderung

- RGB565-Farbtabelle auf vier Bits pro Eintrag gepackt:32768 statt65536B. Alle sechs Farbwerte passen hinein. Initialisierung und Nachschlagen ergeben genau dieselbe Klassifikation, ohne neue HSV-Grenzen oder groessere/kleinere Detektionsfenster. Logpuffer bleibt192KiB; keine verlorene Kapazitaet, keine geaenderte Fahrgeschwindigkeit oder Route.
- USBLogger::log_save_stage() meldet ausschliesslich bei verbundenem USB-Terminal und ausserhalb DC_ENABLED einmal pro Phase: power_settle, connect, mount, directory_open, directory_scan, file_open, write_flush, finalize_verify. Ausgabe: `[LOGGER USB] stage=... heap_used/peak/reserved/fails=...`. Die Mbed-Zahlen werden roh und ohne unbewiesene Freispeicherberechnung ausgegeben. Keine dynamische Diagnoseallokation, kein zusaetzliches Loggen waehrend Fahrt.
- Die Phase ist der zuletzt erreichte Aufrufbereich, kein vollstaendiger Crashstack. Der parallele USB-Worker kann ebenfalls abstuerzen. Es wurde kein fatal-error-hook hinzugefuegt, der im Fehlerkontext riskante USB-Ausgaben ausfuehren koennte.
- Schreib-/Flush-/Abschlusskontrolle und vollstaendiger Wiederholversuch in eine neue Datei bleiben erhalten; RAM-Prefix wird erst bei Erfolg entfernt.

Neues ELF: __end__=0x2406e638, gleiches __HeapLimit, somit71112B Heap-Spanne. Netto32760B mehr;8B Zusatzdaten fuer Diagnose/Linklayout. Statisches RAM452136/523624B, Flash598048/786432B.

## Pruefungen

- check_packed_vision_lookup.py: alle65536 RGB565-Werte gegen die bestehende RGB->HSV->Farbentscheidung, zweimal nach begin():131072 bitgenaue Vergleiche bestanden. Anfangs fehlte dem separaten Host-Fixture die vollstaendige Arduino-Typoberflaeche; Fixture ergaenzt, Firmwarecode dafuer nicht geaendert.
- check_surprise_rule.py:45/45 freigegebene aktuelle Blockbilder,16 reale Bildfolgen,648 gerenderte/48 Zuordnungs-/24 Regel-/Fehlerfaelle bestanden; Sonderfarbe bleibt bis Runde3 gespeichert.
- check_green_seat_visibility.py: echte freigegebene Archiv-Abfragen und synthetische Hintergrund-/Belegungs-/Fussgate-Negativkontrollen bestanden.
- check_far_obstacle_acquisition.py:52 Faelle bestanden.
- check_logger_directory_save.py:546 alte Dateien, begrenzte16er-Verzeichnisbearbeitung, erfolgreicher kompletter Snapshot, korrekte Phasenfolge, kein Fortschritt waehrend Fahrt, Short-write-Wiederholung in neue Datei und RAM-Erhalt bei Verzeichnisfehler bestehen. USB-/Mbed-Host ist dabei gemockt; dies reproduziert den physischen Absturz nicht.

IDE-PlatformIO M7 SUCCESS; bestehende Serial/USB-Makro-/Legacy-Warnungen, keine Fehler. M4 unveraendert und nicht gebaut. Binary601600B SHA256: 69afd090f0d0347d930125a1b52f9c2734b339a9f7997ef720822f9ffd9d8882. Keine Uebertragung, kein weiterer Commit.

## Exakter naechster Nachweis

Nutzer spielt selbst nur giga_r1_m7 auf. Fuer den ersten Speicherversuch Fahrfreigabe aus und USB-Terminal115200 bereits vor dem normalen Speichern geoeffnet lassen; kein Fahr-/ToF-Sondertest erforderlich. Letzte LOGGER-USB-Phase, Heapzahlen, eventuelle Mbed-Ausgabe und LED-Verhalten sichern. Ein erfolgreicher Versuch entlastet den Speicherverdacht, beweist aber keine universelle Zuverlaessigkeit; bei erneutem Absturz benennt die letzte Phase den weiter zu pruefenden Bereich. Stickdatei sofort vollstaendig sichern und Quelle/Kopie hashen. Keine Formatierung/ungepruefte Treiberwechsel/Fahrparameteraenderung.

## Physischer Folgetest bestaetigt

Im stehenden Versuch erreicht die neue Version alle USB-Phasen, meldet71112B Heapkapazitaet und0 Allokationsfehler. Mount benoetigt42432B belegten Heap, Schreiben44484B Spitze; beides uebersteigt den alten gesamten38352B-Bereich. Nach dem Speichern antwortet die Anwendung auf die reine Positionsabfrage. Auf dem Stick liegt eine neue, vollstaendige930B-Datei log_1.txt mit dem erwarteten Initialisierungs-/SYSTEM-DISABLED-Snapshot. Original, SHA256 und Kontext: [stationaerer Speichernachweis](evidence/log_intake/20261119_stationary_usb_save/README.md). Dies ist ein belastbarer neuer Speichernachweis und starke Evidenz fuer den frueheren Speicherengpass; kein alter Crashstack und kein kompletter Challenge-/Langlogtest.
