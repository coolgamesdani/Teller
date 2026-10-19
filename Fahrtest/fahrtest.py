#!/usr/bin/env python3
"""Fahrtest mit Lidar: der Teller faehrt nacheinander in alle Richtungen, danach
wird aus den Lidar-Scans berechnet, wie weit und wohin er wirklich gefahren ist.

Auf dem Teller muss Fahrtest.ino laufen.

  python3 fahrtest.py aufnahme PORT [--tempo 60] [--dauer 500] [--ja]
  python3 fahrtest.py auswerten aufnahmen/aufnahmeN.bin

Auswertung: In jeder Pause steht der Teller still und der Lidar sieht den Raum
mehrmals rundum. Die Punktwolke nach einer Fahrt wird so verschoben und gedreht,
dass sie auf die Wolke davor passt (Scan-Matching, ICP). Diese Verschiebung ist
die wirklich gefahrene Strecke.

Koordinaten des Tellers: x = vorne, y = links, Drehung > 0 = nach links
(gegen den Uhrzeigersinn), alles bezogen auf die Lage vor der jeweiligen Fahrt.
"""
import argparse
import glob
import math
import os
import sys
import threading
import time

import numpy as np
from scipy.spatial import cKDTree

ORDNER = os.path.dirname(os.path.abspath(__file__))
AUFNAHMEN = os.path.join(ORDNER, "aufnahmen")
AUSWERTUNG = os.path.join(ORDNER, "auswertung")

PAKET_LEN = 47
# Wie in Teller.ino: Winkel, den der Lidar meldet, wenn er genau nach vorne schaut.
# Der Lidar zaehlt im Uhrzeigersinn.
LIDAR_MOUNT_DEG = 270.0
# Lage des Lidars relativ zur Drehmitte des Tellers (x vorne, y links), in m.
# Aus vier Drehfahrten bestimmt (aufnahme2/3, Restfehler < 1 cm). Damit beziehen
# sich alle Ergebnisse auf die Tellermitte statt auf den Lidar.
LIDAR_POS = (0.098, 0.002)
MIN_ABSTAND = 0.15   # m, naeher ist Teller/Kabel
MAX_ABSTAND = 8.0    # m
MIN_INTENSITAET = 10
NACHLAUF_PAKETE = 130  # nach dem Anhalten verwerfen (~0,35 s), bis der Teller ruht
RASTER = 0.02        # m, Punktwolke auf dieses Raster ausduennen

# (Name, vx, vy, omega) - Werte wie vom Joystick, Tempo wird draufmultipliziert.
# Je zwei Fahrten heben sich auf, damit der Teller in der Naehe des Starts bleibt.
# Laut Teller.ino: vx > 0 vorwaerts, omega > 0 dreht nach rechts. Fuer vy > 0
# erwarte ich "nach rechts"; das prueft dieser Test gleich mit.
SCHRITTE = [
    ("vor",              1,  0,  0),
    ("zurueck",         -1,  0,  0),
    ("rechts",           0,  1,  0),
    ("links",            0, -1,  0),
    ("vorne rechts",     1,  1,  0),
    ("hinten links",    -1, -1,  0),
    ("vorne links",      1, -1,  0),
    ("hinten rechts",   -1,  1,  0),
    ("drehen rechts",    0,  0,  1),
    ("drehen links",     0,  0, -1),
]


def erwartet(vx, vy, om):
    """Erwartete Fahrtrichtung im Teller-Koordinatensystem (x vorne, y links)."""
    return np.array([vx, -vy], dtype=float), -om


# ------------------------------------------------------------------ Aufnahme

def port_oeffnen(port):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 921600
    ser.timeout = 0.05
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    return ser


def aufnahme(args):
    tempo, dauer = args.tempo, args.dauer
    print("Ablauf (Tempo %d %%, je %d ms, Drehungen %d ms):" % (tempo, dauer, args.dreh_dauer))
    for i, (name, vx, vy, om) in enumerate(SCHRITTE, 1):
        print("  %2d  %-15s vx=%4d vy=%4d omega=%4d" % (i, name, vx * tempo, vy * tempo, om * tempo))
    if not args.ja:
        input("Teller auf dem Boden, Kabel locker, niemand im Raum bewegt sich? [Enter = los, Strg+C = abbrechen] ")

    ser = port_oeffnen(args.port)
    daten = bytearray()
    stop = threading.Event()

    def lesen():
        while not stop.is_set():
            daten.extend(ser.read(4096))

    leser = threading.Thread(target=lesen, daemon=True)
    leser.start()
    try:
        # Der ESP32 startet beim Oeffnen des Ports neu; warten, bis Lidar-Daten kommen.
        ende = time.time() + 6
        while time.time() < ende and b"# bereit" not in daten:
            time.sleep(0.1)
        time.sleep(2.0)  # Lidar auf Drehzahl, erster Stillstands-Scan
        if daten.count(b"\x54\x2c") < 200:
            sys.exit("Keine Lidar-Daten. Laeuft Fahrtest.ino, ist der Lidar angeschlossen und an?")
        for i, (name, vx, vy, om) in enumerate(SCHRITTE, 1):
            print("  %2d/%d %s" % (i, len(SCHRITTE), name), flush=True)
            ms = args.dreh_dauer if om else dauer
            ser.write(b"f %d %d %d %d\n" % (vx * tempo, vy * tempo, om * tempo, ms))
            time.sleep(ms / 1000 + 1.5)
    except KeyboardInterrupt:
        ser.write(b"s\n")
        print("\nAbgebrochen, Teller angehalten.")
    finally:
        ser.write(b"s\n")
        time.sleep(0.2)
        stop.set()
        leser.join()
        ser.close()

    os.makedirs(AUFNAHMEN, exist_ok=True)
    nr = 1 + max([int(os.path.basename(f)[8:-4]) for f in glob.glob(os.path.join(AUFNAHMEN, "aufnahme*.bin"))] or [0])
    pfad = os.path.join(AUFNAHMEN, "aufnahme%d.bin" % nr)
    with open(pfad, "wb") as f:
        f.write(daten)
    print("Gespeichert: %s (%d kB)" % (pfad, len(daten) // 1024))
    auswerten(pfad)


# ------------------------------------------------------------------ Einlesen

def crc_tabelle():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC = crc_tabelle()


def crc8(b):
    c = 0
    for x in b:
        c = CRC[c ^ x]
    return c


def zerlegen(daten):
    """Liefert eine Liste von ("p", paket) und ("t", textzeile) in Empfangsreihenfolge."""
    ereignisse, i, n = [], 0, len(daten)
    while i < n:
        if daten[i] == 0x54 and i + PAKET_LEN <= n and daten[i + 1] == 0x2C \
                and crc8(daten[i:i + PAKET_LEN - 1]) == daten[i + PAKET_LEN - 1]:
            ereignisse.append(("p", bytes(daten[i:i + PAKET_LEN])))
            i += PAKET_LEN
        elif daten[i] == ord("#"):
            j = daten.find(b"\n", i)
            if j < 0:
                break
            ereignisse.append(("t", daten[i:j].decode("ascii", "replace")))
            i = j + 1
        else:
            i += 1
    return ereignisse


def punkte(pakete):
    """Lidar-Pakete -> Punkte im Teller-Koordinatensystem (x vorne, y links), in m."""
    xy = []
    for p in pakete:
        start = (p[4] | p[5] << 8) / 100.0
        ende = (p[42] | p[43] << 8) / 100.0
        if ende < start:
            ende += 360.0
        schritt = (ende - start) / 11
        for k in range(12):
            d = (p[6 + 3 * k] | p[7 + 3 * k] << 8) / 1000.0
            if not (MIN_ABSTAND <= d <= MAX_ABSTAND) or p[8 + 3 * k] < MIN_INTENSITAET:
                continue
            # Winkel im Uhrzeigersinn ab "vorne"
            a = math.radians(start + schritt * k - LIDAR_MOUNT_DEG)
            xy.append((LIDAR_POS[0] + d * math.cos(a), LIDAR_POS[1] - d * math.sin(a)))
    xy = np.array(xy) if xy else np.zeros((0, 2))
    if len(xy):
        _, idx = np.unique(np.round(xy / RASTER).astype(int), axis=0, return_index=True)
        xy = xy[np.sort(idx)]
    return xy


def abschnitte(ereignisse):
    """Teilt die Aufnahme in Stillstands-Wolken und Fahrten auf.

    Liefert (wolken, fahrten): wolken[i] ist der Stillstand vor Fahrt i,
    wolken[-1] der nach der letzten Fahrt; fahrten[i] = (vx, vy, omega, ms).
    """
    wolken, fahrten, aktuell, verwerfen = [], [], [], 0
    for art, inhalt in ereignisse:
        if art == "p":
            if verwerfen > 0:
                verwerfen -= 1
            else:
                aktuell.append(inhalt)
        elif inhalt.startswith("# f "):
            teile = inhalt.split()
            wolken.append(punkte(aktuell))
            fahrten.append(tuple(int(x) for x in teile[2:6]))
            aktuell = []
            verwerfen = 10 ** 9  # waehrend der Fahrt nichts sammeln
        elif inhalt.startswith(("# e ", "# s ")) and verwerfen > 0:
            aktuell, verwerfen = [], NACHLAUF_PAKETE
    wolken.append(punkte(aktuell))
    return wolken, fahrten


# ------------------------------------------------------------------ Scan-Matching

def drehung(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def normalen(wolke, baum, k=8):
    _, idx = baum.query(wolke, k=min(k, len(wolke)))
    n = np.zeros_like(wolke)
    for i, nb in enumerate(idx):
        q = wolke[nb] - wolke[nb].mean(axis=0)
        w, v = np.linalg.eigh(q.T @ q)
        n[i] = v[:, 0]  # kleinster Eigenwert = senkrecht zur Wand
    return n


def icp(quelle, ziel, baum, n_ziel, start):
    """Punkt-zu-Linie-ICP. Sucht (theta, t) mit drehung(theta) @ quelle + t ~ ziel."""
    theta, t = start[0], np.array(start[1:], dtype=float)
    for schwelle in (0.5, 0.3, 0.2, 0.12, 0.08, 0.05, 0.05, 0.05):
        for _ in range(6):
            p = quelle @ drehung(theta).T + t
            d, idx = baum.query(p)
            m = d < schwelle
            if m.sum() < 30:
                return None
            pm, n = p[m], n_ziel[idx[m]]
            r = np.einsum("ij,ij->i", pm - ziel[idx[m]], n)
            rel = pm - t
            J = np.column_stack([n[:, 0] * -rel[:, 1] + n[:, 1] * rel[:, 0], n[:, 0], n[:, 1]])
            # Huber-Gewichte: einzelne Ausreisser (Menschen, Kabel) ziehen weniger.
            w = np.minimum(1.0, 0.02 / np.maximum(np.abs(r), 1e-9))
            dx = np.linalg.lstsq(J * w[:, None], -r * w, rcond=None)[0]
            theta += dx[0]
            t += dx[1:]
            if np.abs(dx).max() < 1e-5:
                break
    p = quelle @ drehung(theta).T + t
    d, _ = baum.query(p)
    passend = d < 0.05
    return theta, t, passend.mean(), float(np.sqrt(np.mean(d[passend] ** 2))) if passend.any() else 1.0


def bewegung(vorher, nachher, richtung=None):
    """Lage nach der Fahrt im Koordinatensystem vor der Fahrt: (theta, t, Anteil passend, RMS)."""
    baum = cKDTree(vorher)
    n = normalen(vorher, baum)
    starts = []
    # Bei Fahrten in eine Richtung zuerst kleine Drehungen mit Weg in Sollrichtung,
    # danach (und bei Drehungen nur) der volle Kreis in 15-Grad-Schritten.
    if richtung is not None:
        for winkel in (0, 20, -20, 45, -45):
            for weg in (0.0, 0.15, 0.3, 0.5):
                v = richtung / max(np.linalg.norm(richtung), 1e-9) * weg
                starts.append((math.radians(winkel), v[0], v[1]))
    starts += [(math.radians(w), 0.0, 0.0) for w in range(-180, 180, 15)]
    best = None
    for s in starts:
        e = icp(nachher, vorher, baum, n, s)
        if e and (best is None or e[2] > best[2] + 0.02 or (abs(e[2] - best[2]) <= 0.02 and e[3] < best[3])):
            best = e
        if best and best[2] > 0.95 and best[3] < 0.02:
            break  # passt eindeutig, weitere Startwerte sparen
    return best


# ------------------------------------------------------------------ Auswertung

def winkel_deg(v):
    return math.degrees(math.atan2(v[1], v[0]))


def richtungswort(v):
    if np.linalg.norm(v) < 0.02:
        return "steht"
    namen = ["vor", "vorne links", "links", "hinten links", "zurueck", "hinten rechts", "rechts", "vorne rechts"]
    return namen[int(round(winkel_deg(v) / 45.0)) % 8]


def auswerten(pfad):
    with open(pfad, "rb") as f:
        daten = f.read()
    ereignisse = zerlegen(daten)
    texte = [e[1] for e in ereignisse if e[0] == "t"]
    n_pakete = sum(1 for e in ereignisse if e[0] == "p")
    wolken, fahrten = abschnitte(ereignisse)
    print("\n%s: %d Lidar-Pakete, %d Fahrten, %d x 'kein lidar'"
          % (os.path.basename(pfad), n_pakete, len(fahrten), sum(t.startswith("# kein") for t in texte)))
    if not fahrten:
        sys.exit("Keine Fahrten in der Aufnahme.")

    namen = {(vx, vy, om): name for name, vx, vy, om in SCHRITTE}
    ergebnisse, lage = [], [(0.0, np.zeros(2))]
    print("\n Nr  Befehl          Punkte     gefahren  Richtung (Soll)                  Abweichung   Drehung  passend")
    for i, (vx, vy, om, ms) in enumerate(fahrten):
        vorher, nachher = wolken[i], wolken[i + 1]
        tempo = max(abs(vx), abs(vy), abs(om), 1)
        name = namen.get((int(np.sign(vx)), int(np.sign(vy)), int(np.sign(om))), "%d/%d/%d" % (vx, vy, om))
        soll, soll_dreh = erwartet(vx / tempo, vy / tempo, om / tempo)
        if len(vorher) < 50 or len(nachher) < 50:
            print(" %2d  %-15s  zu wenige Punkte (%d/%d)" % (i + 1, name, len(vorher), len(nachher)))
            ergebnisse.append(None)
            continue
        e = bewegung(vorher, nachher, soll if np.linalg.norm(soll) else None)
        if e is None:
            print(" %2d  %-15s  Scan-Matching fehlgeschlagen" % (i + 1, name))
            ergebnisse.append(None)
            continue
        theta, t, anteil, rms = e
        ergebnisse.append(e)
        th0, t0 = lage[-1]
        lage.append((th0 + theta, t0 + drehung(th0) @ t))
        weg = np.linalg.norm(t) * 100
        if np.linalg.norm(soll):
            abw = (winkel_deg(t) - winkel_deg(soll) + 180) % 360 - 180
            richtung = "%-13s (%s" % (richtungswort(t), richtungswort(soll) + ")")
            abw_text = "%+6.1f deg" % abw
        elif abs(math.degrees(theta)) > 150:
            # Aus zwei Stillstands-Scans ist 180 Grad rechts nicht von 180 Grad links zu
            # unterscheiden, in einem fast rechteckigen Raum auch nicht von 0 Grad.
            richtung = "%-13s (%s" % ("~180 deg ??", "dreht " + ("links" if soll_dreh > 0 else "rechts") + ")")
            abw_text = "      -  "
        else:
            richtung = "%-13s (%s" % ("dreht " + ("links" if theta > 0 else "rechts"),
                                      "dreht " + ("links" if soll_dreh > 0 else "rechts") + ")")
            abw_text = "      -  "
        print(" %2d  %-15s %4d/%-4d %6.1f cm  %-32s %s  %+7.1f deg  %3.0f %%  (RMS %.1f cm)"
              % (i + 1, name, len(vorher), len(nachher), weg, richtung, abw_text,
                 math.degrees(theta), anteil * 100, rms * 100))

    # Gegenprobe: letzte Wolke direkt gegen die erste
    e = bewegung(wolken[0], wolken[-1])
    if e:
        th_k, t_k = lage[-1]
        print("\nAm Ende gegenueber dem Start: %.1f cm, %+.1f deg  (aufsummiert: %.1f cm, %+.1f deg)"
              % (np.linalg.norm(e[1]) * 100, math.degrees(e[0]), np.linalg.norm(t_k) * 100, math.degrees(th_k)))

    zusammenfassung(fahrten, ergebnisse)
    bild(pfad, wolken, fahrten, ergebnisse, lage)


def zusammenfassung(fahrten, ergebnisse):
    print("\nZusammenfassung:")
    auffaellig = 0
    for i, ((vx, vy, om, ms), e) in enumerate(zip(fahrten, ergebnisse)):
        if e is None:
            continue
        theta, t = e[0], e[1]
        tempo = max(abs(vx), abs(vy), abs(om), 1)
        soll, soll_dreh = erwartet(vx / tempo, vy / tempo, om / tempo)
        if np.linalg.norm(soll):
            quer = abs(np.cross(soll / np.linalg.norm(soll), t)) * 100
            if np.linalg.norm(t) < 0.02:
                print("  Fahrt %d: hat sich praktisch nicht bewegt" % (i + 1))
            elif np.dot(soll, t) < 0:
                print("  Fahrt %d: faehrt in die GEGENRICHTUNG (Vorzeichen pruefen)" % (i + 1))
            elif abs(math.degrees(theta)) > 3 or quer > 0.10 * np.linalg.norm(t) * 100:
                print("  Fahrt %d: %.1f cm quer zur Sollrichtung, %+.1f deg verdreht" % (i + 1, quer, math.degrees(theta)))
            else:
                continue
        elif abs(math.degrees(theta)) > 150:
            print("  Fahrt %d: Drehung um etwa 180 deg, Richtung und Betrag nicht eindeutig messbar" % (i + 1))
        elif soll_dreh * theta < 0:
            print("  Fahrt %d: dreht in die GEGENRICHTUNG (Vorzeichen pruefen)" % (i + 1))
        elif abs(math.degrees(theta)) < 5:
            print("  Fahrt %d: hat sich kaum gedreht" % (i + 1))
        else:
            continue
        auffaellig += 1
    if not auffaellig:
        print("  alle Fahrten innerhalb 3 deg Drehung und 10 % seitlichem Versatz")


def bild(pfad, wolken, fahrten, ergebnisse, lage):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(fahrten)
    spalten = 4
    zeilen = 1 + (n + spalten - 1) // spalten
    fig = plt.figure(figsize=(4 * spalten, 4 * zeilen))

    # Oben: Raum aus der ersten Wolke und alle Lagen des Tellers
    ax = fig.add_subplot(zeilen, 1, 1)
    w = wolken[0]
    ax.scatter(w[:, 0], w[:, 1], s=1, c="0.6")
    for k, (th, t) in enumerate(lage):
        ax.annotate("", xy=t + 0.12 * np.array([math.cos(th), math.sin(th)]), xytext=t,
                    arrowprops=dict(arrowstyle="->", color="C3"))
        ax.text(t[0], t[1], str(k), fontsize=8, color="C3")
    xy = np.array([t for _, t in lage])
    ax.plot(xy[:, 0], xy[:, 1], "-", color="C3", lw=1)
    ax.set_aspect("equal")
    ax.set_title("Raum beim Start (grau) und Lage des Tellers nach jeder Fahrt (rot, Pfeil = vorne)")
    ax.set_xlabel("x vorne (m)")
    ax.set_ylabel("y links (m)")

    # Darunter: pro Fahrt die Wolke davor (grau) und danach, eingepasst (farbig)
    for i, ((vx, vy, om, ms), e) in enumerate(zip(fahrten, ergebnisse)):
        ax = fig.add_subplot(zeilen, spalten, spalten + i + 1)
        a, b = wolken[i], wolken[i + 1]
        ax.scatter(a[:, 0], a[:, 1], s=1, c="0.6")
        titel = "%d: %d/%d/%d" % (i + 1, vx, vy, om)
        if e is not None:
            p = b @ drehung(e[0]).T + e[1]
            ax.scatter(p[:, 0], p[:, 1], s=1, c="C0")
            ax.annotate("", xy=e[1], xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="C3", lw=2))
            titel += "  %.0f cm, %+.0f deg" % (np.linalg.norm(e[1]) * 100, math.degrees(e[0]))
        ax.set_title(titel, fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)

    fig.tight_layout()
    os.makedirs(AUSWERTUNG, exist_ok=True)
    ziel = os.path.join(AUSWERTUNG, os.path.splitext(os.path.basename(pfad))[0] + ".png")
    fig.savefig(ziel, dpi=110)
    print("\nBild: %s" % ziel)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="befehl", required=True)
    a = sub.add_parser("aufnahme")
    a.add_argument("port")
    a.add_argument("--tempo", type=int, default=60, help="Prozent wie am Joystick (Standard 60)")
    a.add_argument("--dauer", type=int, default=500, help="ms pro Fahrt (Standard 500)")
    a.add_argument("--dreh-dauer", type=int, default=200,
                   help="ms pro Drehung (Standard 200). Ueber ~150 Grad ist die Richtung nicht mehr eindeutig.")
    a.add_argument("--ja", action="store_true", help="ohne Rueckfrage starten")
    b = sub.add_parser("auswerten")
    b.add_argument("datei")
    args = p.parse_args()
    if args.befehl == "aufnahme":
        aufnahme(args)
    else:
        auswerten(args.datei)


if __name__ == "__main__":
    main()
