"""Dauerfahrt mit Live-Mitschnitt (USB-Kabel bleibt am Teller).

  python3 live.py PORT AUSGABE.csv          startet die Linienfahrt und schreibt mit
  python3 live.py werte AUSGABE.csv         wertet den Mitschnitt je Durchfahrt aus

Waehrend live.py laeuft, wird der Inhalt von AUSGABE.csv.cmd (falls vorhanden)
an den Teller geschickt und die Datei geloescht, z. B. "p turn 0.4".
AUSGABE.csv.stop beendet die Fahrt.
"""
import os, sys, time
import serial

W, B = None, None
PHASEN = ["folgen", "ecke-halt", "ecke-drehen", "ecke-steht", "aufgegeben", "ziel",
          "wende-halt", "wenden", "wende-steht", "einparken", "warten", "losfahren", "ausrichten"]


def fahren(port, pfad):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.05
    s.dtr = s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    out = open(pfad, "a")
    info = open(pfad + ".log", "a")
    s.write(b"v")  # Live-Ausgabe an
    s.write(b"p\n")
    s.write(b"g")
    letzt = 0
    puffer = b""
    try:
        while not os.path.exists(pfad + ".stop"):
            if time.time() - letzt > 0.25:
                s.write(b"k")
                letzt = time.time()
            if os.path.exists(pfad + ".cmd"):
                cmd = open(pfad + ".cmd").read().strip()
                os.remove(pfad + ".cmd")
                for zeile in cmd.splitlines():
                    s.write((zeile.strip() + "\n").encode() if zeile.startswith("p") else zeile.strip().encode())
                    info.write("%.2f > %s\n" % (time.time(), zeile)); info.flush()
            puffer += s.read(4096)
            *zeilen, puffer = puffer.split(b"\n")
            for z in zeilen:
                z = z.decode("ascii", "replace").strip()
                if z.startswith("@"):
                    out.write("%.3f,%s\n" % (time.time(), z[1:]))
                elif z:
                    info.write("%.2f %s\n" % (time.time(), z))
            out.flush(); info.flush()
    finally:
        for _ in range(3):
            s.write(b"s"); time.sleep(0.05)
        s.write(b"v")
        s.close()


def werte(pfad):
    import re
    text = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Teller.ino")).read()
    w = [int(x) for x in re.search(r"LIGHT_WHITE\[3\] = \{([^}]*)\}", text).group(1).split(",")]
    b = [int(x) for x in re.search(r"LIGHT_BLACK\[3\] = \{([^}]*)\}", text).group(1).split(",")]
    dk = lambda r, i: min(1.0, max(0.0, (w[i] - r[3 + i]) / (w[i] - b[i])))
    rows = []
    for l in open(pfad):
        t = l.strip().split(",")
        if len(t) == 10:
            rows.append([float(t[0])] + [float(x) for x in t[1:]])
    # Durchfahrten: von "folgen" nach einer Wende bis zur naechsten Wende
    fahrten, cur = [], []
    for r in rows:
        if int(r[2]) == 6 and cur:
            fahrten.append(cur); cur = []
        cur.append(r)
    if cur:
        fahrten.append(cur)
    for n, f in enumerate(fahrten):
        fol = [r for r in f if int(r[2]) == 0 and r[7] > 0]
        if len(fol) < 10:
            continue
        mitte = sum(dk(r, 1) >= 0.5 for r in fol) / len(fol)
        fehler = sum(abs(dk(r, 2) - dk(r, 0)) for r in fol) / len(fol)
        wechsel = sum(1 for a, c in zip(fol, fol[1:]) if a[9] * c[9] < 0 and abs(a[9] - c[9]) > 30)
        dauer = fol[-1][1] / 1000 - fol[0][1] / 1000
        such = sum(1 for r in fol if r[7] == 0) / len(fol)
        phasen = sorted({PHASEN[int(r[2])] for r in f})
        print("Fahrt %2d  %5.1f s  Mitte auf Linie %3d%%  |Fehler| %.2f  Lenkwechsel %.1f/s  %s"
              % (n, dauer, 100 * mitte, fehler, wechsel / max(dauer, 0.1),
                 "aufgegeben!" if "aufgegeben" in phasen else ""))


if __name__ == "__main__":
    if sys.argv[1] == "werte":
        werte(sys.argv[2])
    else:
        fahren(sys.argv[1], sys.argv[2])
