#!/usr/bin/env python3
"""Eine Geradeaus-Testfahrt starten und auswerten (auf dem Teller laeuft Geradeaus.ino).

  python3 fahrt.py PORT VX DAUER_MS MAX1 MAX2 MAX3 MAX4     Fahrt
  python3 fahrt.py PORT                                     nur Sensoren lesen

Fehler e = dunkel(R) - dunkel(L): > 0 heisst, die Linie liegt rechts vom Teller,
er ist also nach links abgewichen.
"""
import glob
import os
import statistics
import sys
import time

import serial

WHITE = [466, 653, 225]
BLACK = [85, 53, 8]
ORDNER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fahrten")


def dunkel(i, roh):
    return min(1.0, max(0.0, (WHITE[i] - roh) / (WHITE[i] - BLACK[i])))


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
    ser.reset_input_buffer()
    return ser


def main():
    ser = port_oeffnen(sys.argv[1])
    if len(sys.argv) < 8:
        ser.write(b"r\n")
        zeile = ser.readline().decode("ascii", "replace").split()
        roh = [int(x) for x in zeile[2:5]]
        d = [dunkel(i, roh[i]) for i in range(3)]
        print("roh L/M/R %s   dunkel %.2f %.2f %.2f   e=%+.2f" % (roh, d[0], d[1], d[2], d[2] - d[0]))
        return

    ser.write(("g " + " ".join(sys.argv[2:8]) + "\n").encode())
    zeilen, kopf, ende = [], "", "keine Antwort"
    t0 = time.time()
    while time.time() - t0 < 6:
        zeile = ser.readline().decode("ascii", "replace").strip()
        if zeile.startswith("# start"):
            kopf = zeile
        elif zeile.startswith("# ende"):
            ende = zeile[7:]
            break
        else:
            teile = zeile.split()
            if len(teile) == 4 and all(t.isdigit() for t in teile):
                zeilen.append([int(t) for t in teile])
    ser.close()
    if not zeilen:
        sys.exit("Keine Messwerte (%s)" % ende)

    os.makedirs(ORDNER, exist_ok=True)
    nr = 1 + max([int(os.path.basename(f)[5:-4]) for f in glob.glob(os.path.join(ORDNER, "fahrt*.csv"))] or [0])
    pfad = os.path.join(ORDNER, "fahrt%d.csv" % nr)
    with open(pfad, "w") as f:
        f.write(kopf + "\n# ende " + ende + "\nms,L,M,R,e\n")
        for t, l, m, r in zeilen:
            f.write("%d,%d,%d,%d,%+.2f\n" % (t, l, m, r, dunkel(2, r) - dunkel(0, l)))

    print("%s -> fahrt%d.csv, %d Werte, %d ms, Ende: %s" % (kopf, nr, len(zeilen), zeilen[-1][0], ende))
    print("   ms     dL    dM    dR      e")
    schritt = 100
    for a in range(0, zeilen[-1][0] + 1, schritt):
        teil = [z for z in zeilen if a <= z[0] < a + schritt]
        if not teil:
            continue
        # Median statt Mittel: einzelne Ausreisser der Sensoren fallen heraus.
        d = [statistics.median(dunkel(i, z[i + 1]) for z in teil) for i in range(3)]
        print("%5d   %.2f  %.2f  %.2f  %+.2f" % (a, d[0], d[1], d[2], d[2] - d[0]))


if __name__ == "__main__":
    main()
