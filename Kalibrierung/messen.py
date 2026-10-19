#!/usr/bin/env python3
"""Kalibriermessung der drei Lichtsensoren des Tellers.

Auf dem Teller muss Kalibrierung.ino laufen. Das Skript fuehrt durch die vier
Faelle (weiss/schwarz, mit/ohne Umgebungslicht), misst jeweils ein paar
Sekunden und haengt die Ergebnisse an messwerte.csv an.

  python3 messen.py [PORT] [--sekunden 3]
"""
import argparse
import csv
import datetime
import os
import statistics
import sys
import time

import serial

SENSOREN = ["L", "M", "R"]
CSV_DATEI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "messwerte.csv")
CSV_KOPF = ["zeit", "untergrund", "umgebungslicht", "sensor",
            "mittel", "min", "max", "stdabw", "anzahl"]


def port_oeffnen(port):
    # DTR/RTS aus, sonst haelt der USB-Wandler den ESP32 im Reset.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    time.sleep(0.3)
    return ser


def messen(ser, sekunden):
    """Liest `sekunden` lang mit, gibt pro Sensor die Liste der Rohwerte zurueck."""
    ser.reset_input_buffer()
    ser.readline()  # angebrochene Zeile verwerfen
    werte = [[], [], []]
    ende = time.time() + sekunden
    while time.time() < ende:
        teile = ser.readline().decode("ascii", "replace").split()
        if len(teile) != 3:
            continue
        try:
            zahlen = [int(t) for t in teile]
        except ValueError:
            continue
        for i in range(3):
            werte[i].append(zahlen[i])
    return werte


def auswerten(liste):
    return {
        "mittel": statistics.fmean(liste),
        "min": min(liste),
        "max": max(liste),
        "stdabw": statistics.pstdev(liste),
        "anzahl": len(liste),
    }


def speichern(zeilen):
    neu = not os.path.exists(CSV_DATEI)
    with open(CSV_DATEI, "a", newline="") as f:
        w = csv.writer(f)
        if neu:
            w.writerow(CSV_KOPF)
        w.writerows(zeilen)


def schritt(ser, sekunden, untergrund, licht, sensoren, ergebnisse):
    """Eine Messung. `sensoren` = Indizes, die gerade ueber dem Untergrund stehen."""
    namen = "alle drei Sensoren" if len(sensoren) == 3 else "Sensor " + SENSOREN[sensoren[0]]
    while True:
        eingabe = input("\n>> %s ueber %s, Umgebungslicht %s.  [Enter = messen, u = ueberspringen] "
                        % (namen, untergrund.upper(), licht.upper())).strip().lower()
        if eingabe == "u":
            return
        werte = messen(ser, sekunden)
        if not werte[0]:
            print("   Keine Messwerte empfangen. Laeuft Kalibrierung.ino auf dem Teller?")
            continue
        zeit = datetime.datetime.now().isoformat(timespec="seconds")
        zeilen = []
        for i in sensoren:
            a = auswerten(werte[i])
            hinweis = "   <- 0 dabei: unter dem ADC-Messbereich" if a["min"] == 0 else ""
            print("   %s  Mittel %7.1f   min %4d   max %4d   Streuung %5.1f   (n=%d)%s"
                  % (SENSOREN[i], a["mittel"], a["min"], a["max"], a["stdabw"], a["anzahl"], hinweis))
            zeilen.append([zeit, untergrund, licht, SENSOREN[i], "%.1f" % a["mittel"],
                           a["min"], a["max"], "%.1f" % a["stdabw"], a["anzahl"]])
            ergebnisse[(untergrund, licht, i)] = a
        if input("   uebernehmen? [Enter = ja, w = wiederholen] ").strip().lower() != "w":
            speichern(zeilen)
            return


def zusammenfassung(ergebnisse):
    print("\n================ Zusammenfassung (Mittelwerte) ================")
    print("Sensor  Licht   weiss   schwarz   Abstand   Schwelle (Mitte)")
    for i, name in enumerate(SENSOREN):
        for licht in ("an", "aus"):
            w = ergebnisse.get(("weiss", licht, i))
            s = ergebnisse.get(("schwarz", licht, i))
            if not w or not s:
                continue
            print("  %s     %-4s  %7.1f  %7.1f   %7.1f   %7.1f"
                  % (name, licht, w["mittel"], s["mittel"], w["mittel"] - s["mittel"],
                     (w["mittel"] + s["mittel"]) / 2))

    print("\nFeste Schwelle, die bei Licht an UND aus funktioniert:")
    for i, name in enumerate(SENSOREN):
        weiss = [ergebnisse[k] for k in ergebnisse if k[0] == "weiss" and k[2] == i]
        schwarz = [ergebnisse[k] for k in ergebnisse if k[0] == "schwarz" and k[2] == i]
        if not weiss or not schwarz:
            continue
        # unguenstigste Einzelwerte: dunkelstes Weiss gegen hellstes Schwarz (oder umgekehrt)
        w_min, w_max = min(a["min"] for a in weiss), max(a["max"] for a in weiss)
        s_min, s_max = min(a["min"] for a in schwarz), max(a["max"] for a in schwarz)
        if w_min > s_max:
            print("  %s  weiss >= %d, schwarz <= %d  -> Schwelle %d (Luft %d)"
                  % (name, w_min, s_max, (w_min + s_max) // 2, w_min - s_max))
        elif s_min > w_max:
            print("  %s  schwarz >= %d, weiss <= %d  -> Schwelle %d (Luft %d, Sensor invertiert!)"
                  % (name, s_min, w_max, (s_min + w_max) // 2, s_min - w_max))
        else:
            print("  %s  Bereiche ueberlappen (weiss %d..%d, schwarz %d..%d): keine feste Schwelle"
                  % (name, w_min, w_max, s_min, s_max))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("port", nargs="?", default="/dev/ttyUSB1")
    p.add_argument("--sekunden", type=float, default=3.0, help="Messdauer pro Schritt")
    args = p.parse_args()

    ser = port_oeffnen(args.port)
    probe = messen(ser, 0.5)
    if not probe[0]:
        sys.exit("Keine Messwerte auf %s. Laeuft Kalibrierung.ino auf dem Teller?" % args.port)
    print("Verbunden mit %s, aktuell L M R = %d %d %d"
          % (args.port, probe[0][-1], probe[1][-1], probe[2][-1]))

    alle = input("Passen alle drei Sensoren gleichzeitig auf die schwarze Flaeche? [J/n] ")
    einzeln = alle.strip().lower() == "n"
    schwarz_gruppen = [[0], [1], [2]] if einzeln else [[0, 1, 2]]

    ergebnisse = {}
    # Reihenfolge so, dass das Licht nur einmal umgeschaltet werden muss.
    schritt(ser, args.sekunden, "weiss", "an", [0, 1, 2], ergebnisse)
    for gruppe in schwarz_gruppen:
        schritt(ser, args.sekunden, "schwarz", "an", gruppe, ergebnisse)
    for gruppe in schwarz_gruppen:
        schritt(ser, args.sekunden, "schwarz", "aus", gruppe, ergebnisse)
    schritt(ser, args.sekunden, "weiss", "aus", [0, 1, 2], ergebnisse)
    ser.close()

    zusammenfassung(ergebnisse)
    print("\nGespeichert in %s" % CSV_DATEI)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAbgebrochen.")
