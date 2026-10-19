import sys, time, re, collections, select, serial

port, dauer, csv_pfad = sys.argv[1], float(sys.argv[2]), sys.argv[3]
einstellungen = sys.argv[4] if len(sys.argv) > 4 else ''
ser = serial.Serial()
ser.port = port; ser.baudrate = 115200; ser.timeout = 0.05
ser.dtr = False; ser.rts = False
ser.open()
ser.dtr = False; ser.rts = False
time.sleep(0.5)
ser.reset_input_buffer()

muster = re.compile(r'^(\d+) Linie: (\S+)\s+L=(\d) M=(\d) R=(\d)\s+roh\s*(\d+)/\s*(\d+)/\s*(\d+)(?:\s+e=([+-]?[\d.]+))?')
zeilen = []
try:
    for k in re.findall(r'[VCTHDEP]\d\d', einstellungen):
        ser.write(k.encode()); ser.flush(); time.sleep(0.15)
    ser.write(b'p'); ser.flush()
    t0 = time.time()
    while time.time() - t0 < 0.6:
        l = ser.readline().decode('ascii', 'replace').strip()
        if l: print(l)
    print('>>> Fahrt laeuft - ENTER druecken zum Anhalten <<<', flush=True)
    ser.write(b'g'); ser.flush()
    t0 = time.time(); letzt = 0
    while time.time() - t0 < dauer:
        if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            ser.write(b's'); ser.flush()
            print('# abgebrochen nach %.1f s' % (time.time() - t0))
            break
        if time.time() - letzt > 0.25:
            ser.write(b'k'); ser.flush(); letzt = time.time()
        l = ser.readline().decode('ascii', 'replace').strip()
        if not l: continue
        m = muster.match(l)
        if m:
            zeilen.append(tuple(g if g is not None else '' for g in m.groups()))
            if len(zeilen) >= 15 and all(z[1] == 'ziel' for z in zeilen[-15:]):
                print('# Ziel erreicht nach %.1f s' % (time.time() - t0))
                break
        else: print(l)
except KeyboardInterrupt:
    print('# abgebrochen (Strg+C)')
finally:
    try:
        for _ in range(3):
            ser.write(b's'); ser.flush(); time.sleep(0.05)
        t0 = time.time()
        while time.time() - t0 < 0.4:
            l = ser.readline().decode('ascii', 'replace').strip()
            if l.startswith('#'): print(l)
        ser.close()
    except Exception as e:
        print('Stopp-Kommando fehlgeschlagen:', e)

with open(csv_pfad, 'w') as f:
    f.write('ms,zustand,L_hell,M_hell,R_hell,L_roh,M_roh,R_roh,fehler\n')
    for z in zeilen: f.write(','.join(z) + '\n')
print('%d Messpunkte -> %s' % (len(zeilen), csv_pfad))
if not zeilen: sys.exit()

print('\nVerlauf:')
start = 0
for i in range(1, len(zeilen) + 1):
    if i == len(zeilen) or zeilen[i][1] != zeilen[start][1]:
        t_a, t_e = int(zeilen[start][0]), int(zeilen[i - 1][0])
        print('  %5d..%5d ms  %-11s (%d)' % (t_a, t_e, zeilen[start][1], i - start))
        start = i
anteil = collections.Counter(z[1] for z in zeilen)
print('\nAnteile:', ', '.join('%s %d%%' % (k, round(100 * v / len(zeilen))) for k, v in anteil.most_common()))
for i, n in enumerate('LMR'):
    w = [int(z[5 + i]) for z in zeilen]
    print('  %s roh min %4d  max %4d' % (n, min(w), max(w)))
