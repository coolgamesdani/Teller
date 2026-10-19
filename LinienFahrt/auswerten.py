"""Linienfahrten mit Teller.ino aufzeichnen und auswerten.

  python3 auswerten.py fahrt PORT [--dauer 60]   Fahrt per USB starten (Kabel bleibt dran),
                                                 ENTER haelt an, danach Auswertung
  python3 auswerten.py lesen PORT                Aufzeichnung der letzten Fahrt holen
                                                 (Fahrt vorher am Controller gestartet)
  python3 auswerten.py datei FAHRT.csv           gespeicherte Fahrt nochmal auswerten

Der Teller zeichnet jede Linienfahrt mit 50 Hz auf (siehe FAHRTENSCHREIBER in
Teller.ino). Die Fahrten landen in LinienFahrt/fahrten/.
"""
import argparse, os, re, select, sys, time
import serial

HIER = os.path.dirname(os.path.abspath(__file__))
TELLER = os.path.join(HIER, "..", "Teller.ino")
PHASEN = ["folgen", "ecke-halt", "ecke-drehen", "ecke-steht", "aufgegeben", "ziel",
          "wende-halt", "wenden", "wende-steht", "einparken", "warten", "losfahren", "ausrichten"]


def kalibrierung():
    text = open(TELLER).read()
    weiss = [int(x) for x in re.search(r"LIGHT_WHITE\[3\] = \{([^}]*)\}", text).group(1).split(",")]
    schwarz = [int(x) for x in re.search(r"LIGHT_BLACK\[3\] = \{([^}]*)\}", text).group(1).split(",")]
    return weiss, schwarz


def port_oeffnen(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.1
    s.dtr = s.rts = False  # sonst startet der ESP32 neu und die Aufzeichnung ist weg
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def holen(s):
    s.write(b"d")
    zeilen, t0 = [], time.time()
    while time.time() - t0 < 60:  # wartet der Teller noch auf den Controller, antwortet er erst nach ein paar s
        l = s.readline().decode("ascii", "replace").strip()
        if not l:
            continue
        if l.startswith("# ende"):
            break
        if l[0].isdigit():
            zeilen.append(l)
    return zeilen


def speichern(zeilen):
    os.makedirs(os.path.join(HIER, "fahrten"), exist_ok=True)
    pfad = os.path.join(HIER, "fahrten", time.strftime("fahrt_%Y%m%d_%H%M%S.csv"))
    with open(pfad, "w") as f:
        f.write("ms,phase,L,M,R,dreh,vx,vy,om\n" + "\n".join(zeilen) + "\n")
    print("%d Messpunkte -> %s" % (len(zeilen), pfad))
    return pfad


def auswerten(pfad):
    weiss, schwarz = kalibrierung()
    daten = []
    for l in open(pfad).read().splitlines()[1:]:
        t = l.split(",")
        if len(t) == 9:
            daten.append([int(t[0]), int(t[1]), int(t[2]), int(t[3]), int(t[4]), float(t[5]),
                          int(t[6]), int(t[7]), int(t[8])])
    if not daten:
        sys.exit("Keine Daten.")
    dunkel = lambda d, i: min(1.0, max(0.0, (weiss[i] - d[2 + i]) / (weiss[i] - schwarz[i])))

    print("\nVerlauf:")
    start = 0
    for i in range(1, len(daten) + 1):
        if i == len(daten) or daten[i][1] != daten[start][1]:
            a, e = daten[start], daten[i - 1]
            extra = ""
            if PHASEN[a[1]] in ("wenden", "wende-steht"):
                extra = "  Lidar %+.0f -> %+.0f Grad" % (a[5], e[5])
            print("  %6.2f..%6.2f s  %-11s%s" % (a[0] / 1000, e[0] / 1000, PHASEN[a[1]], extra))
            start = i

    folgen = [d for d in daten if d[1] == 0 and d[6] > 0]
    if folgen:
        fehler = [dunkel(d, 2) - dunkel(d, 0) for d in folgen]
        wechsel = sum(1 for a, b in zip(folgen, folgen[1:]) if a[8] * b[8] < 0 and abs(a[8] - b[8]) > 20)
        dauer = (folgen[-1][0] - folgen[0][0]) / 1000 or 1
        mitte = sum(1 for d in folgen if dunkel(d, 1) >= 0.5) / len(folgen)
        print("\nLinie folgen: %.1f s, mittlerer |Fehler| %.2f, Lenkwechsel %.1f/s, Mitte auf der Linie %d%%"
              % (dauer, sum(abs(f) for f in fehler) / len(fehler), wechsel / dauer, 100 * mitte))

    wende = [d for d in daten if PHASEN[d[1]] == "wende-steht"]
    if wende:
        print("Wende: steht bei %.0f Grad (Soll 180)" % abs(wende[-1][5]))
    for i, n in enumerate("LMR"):
        w = [d[2 + i] for d in daten]
        print("  %s roh min %4d  max %4d" % (n, min(w), max(w)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("art", choices=["fahrt", "lesen", "datei"])
    ap.add_argument("ziel")
    ap.add_argument("--dauer", type=float, default=60)
    a = ap.parse_args()
    if a.art == "datei":
        auswerten(a.ziel)
        return
    s = port_oeffnen(a.ziel)
    if a.art == "fahrt":
        print(">>> Fahrt laeuft - ENTER zum Anhalten <<<", flush=True)
        s.write(b"g")
        t0 = letzt = time.time()
        try:
            while time.time() - t0 < a.dauer:
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break
                if time.time() - letzt > 0.25:
                    s.write(b"k")
                    letzt = time.time()
                l = s.readline().decode("ascii", "replace").strip()
                if l.startswith("Wende") or l.startswith("Linie: ziel"):
                    print(l)
        finally:
            for _ in range(3):
                s.write(b"s")
                time.sleep(0.05)
        time.sleep(0.3)
        s.reset_input_buffer()
    zeilen = holen(s)
    s.close()
    if zeilen:
        auswerten(speichern(zeilen))
    else:
        print("Keine Aufzeichnung erhalten.")


if __name__ == "__main__":
    main()
