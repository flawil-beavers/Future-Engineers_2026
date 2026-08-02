# PID- und Motorabstimmung

Die Firmware bietet eine geführte serielle Einstellung. `pid help` zeigt alle
Befehle direkt im seriellen Monitor, `pid show` zeigt die momentan aktiven
Werte und `pid export` erzeugt passende Zeilen für `include/config.h`.

## Regleraufbau

- **LOW Kp/Ki** regelt bis zur LOW-Grenze.
- Zwischen **LOW** und **MID** werden die Werte weich interpoliert.
- Zwischen **MID** und **HIGH** werden sie erneut weich interpoliert.
- Oberhalb der HIGH-Grenze gelten die HIGH-Werte.
- **Kp** reagiert sofort auf einen Geschwindigkeitsfehler. Zu wenig Kp wirkt
  kraftlos; zu viel Kp erzeugt schnelle Schwingungen.
- **Ki** beseitigt einen dauerhaft verbleibenden Fehler unter Last. Zu viel Ki
  erzeugt langsames Pulsieren und Überschwingen.
- **Static feedforward** ist die PWM-Grundkraft gegen Reibung.
- **Kv** ist die zusätzliche PWM pro mm/s Sollgeschwindigkeit.
- **Ka** ist die zusätzliche PWM während der Beschleunigung.
- **Acceleration Kp/Ki** korrigiert den Unterschied zwischen gewünschter und
  gemessener Beschleunigung. Diese Werte gelten für alle Geschwindigkeiten.

## Manuell testen

```text
pid test 150
z
```

Der erste Befehl fährt mit 150 mm/s. `z` beendet den Test. Währenddessen zeigt
`P` einen Statusdatensatz mit Solltempo, Isttempo, PWM und aktiven Gains.

Beispiele zum Ändern:

```text
pid set low kp 0.035
pid set low ki 0.020
pid set mid kp 0.080
pid set high ki 0.040
pid set bound low 120
pid set ff static 86
pid show
```

Die Grenzen müssen immer `LOW < MID < HIGH` erfüllen. Änderungen gelten sofort
im RAM und sind nach einem Neustart weg. Nach erfolgreichem Test `pid export`
senden und die ausgegebenen Zeilen in `include/config.h` übernehmen.

## Autotuner

```text
pid tune <Geschwindigkeit> <Basis-PWM> <Relay-Schritt>
```

Beispiel:

```text
pid tune 150 103 15
```

- **Geschwindigkeit** bestimmt, welchem LOW/MID/HIGH-Bereich das Ergebnis
  zugeordnet wird.
- **Basis-PWM** muss den Motor ungefähr mit dieser Geschwindigkeit bewegen.
- **Relay-Schritt** schaltet symmetrisch ober- und unterhalb der Basis-PWM.

Der Tuner misst mehrere stabile Schwingungen, verwirft asymmetrische oder stark
streuende Messungen und berechnet daraus Kp/Ki. Ein gültiges Resultat wird nur
im RAM auf den gewählten Bereich angewendet. Danach immer mit `pid test` prüfen
und erst bei gutem Fahrverhalten mit `pid export` dauerhaft übernehmen.

Empfohlene Reihenfolge:

1. Genügend gerade Strecke schaffen und Enable zunächst ausschalten.
2. Mit `pid test` eine passende Basis-PWM beziehungsweise das Fahrverhalten
   abschätzen.
3. LOW, MID und HIGH einzeln tunen und nach jedem Lauf praktisch testen.
4. Feedforward und Beschleunigungswerte nur ändern, wenn das Problem in allen
   Geschwindigkeitsbereichen auftritt.
5. Die finalen Werte mit `pid export` sichern.

Der Enable-Schalter und `z` stoppen den Test jederzeit. Ein Autotune-Lauf endet
außerdem automatisch bei Zeit- oder Distanzüberschreitung.
