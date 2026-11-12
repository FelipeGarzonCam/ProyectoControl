#!/usr/bin/env python3
# =============================================================================
#  PRUEBAS DEL SERVO PID - Corte 3, Control de Procesos Industriales
#  Se conecta al Arduino UNO (firmware servo_pid_uno.ino), ejecuta las pruebas,
#  guarda los datos y genera las graficas y metricas para el informe.
#
#  Instalacion (una sola vez):   pip install pyserial numpy matplotlib openpyxl
#  Uso:                          python pruebas_servo.py            (detecta el puerto)
#                                python pruebas_servo.py --puerto COM5
#
#  Los resultados quedan en la carpeta  resultados/<fecha>_<prueba>/ :
#     datos*.csv / datos.xlsx   tiempo, referencia, posicion y accion de control
#     metricas.csv              Mp, tr, ts, ess y u maxima de cada escalon
#     figura.png / figura.pdf   grafica lista para el informe (ancho de columna IEEE)
# =============================================================================

import argparse
import csv
import os
import queue
import re
import sys
import threading
import time
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("Falta pyserial:  pip install pyserial")

try:
    import openpyxl
except ImportError:
    openpyxl = None

# ----------------------------------------------------------------- estilo
COL_REF = "#52514e"                              # referencia: gris, discontinua
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6"]
ANCHO_COL = 3.5                                  # pulgadas: una columna IEEE

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8,
    "axes.titlesize": 8,
    "axes.labelsize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "axes.edgecolor": "#8a8984",
    "axes.linewidth": 0.6,
    "axes.grid": True,
    "grid.color": "#e4e3df",
    "grid.linewidth": 0.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "lines.linewidth": 1.2,
    "legend.frameon": False,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
})

ESTADOS = {0: "PARADO", 1: "LAZO_CERRADO", 2: "LAZO_ABIERTO", 3: "FALLA", 4: "BUSCANDO_0", 5: "CALIBRANDO"}
BAUD = 115200


# =============================================================================
#  Comunicacion con el Arduino
# =============================================================================
class Servo:
    """Lee el puerto en segundo plano y separa los tres tipos de linea:
       '#...'          mensajes        -> se muestran en pantalla
       'S,...'         estado (10 Hz)  -> se guardan, no se muestran
       't,ref,th,u'    telemetria      -> se acumulan durante una captura
    """

    def __init__(self, puerto):
        self.ser = serial.Serial(puerto, BAUD, timeout=0.1)
        self.mensajes = queue.Queue()
        self.estado = None            # dict con el ultimo 'S,...'
        self.capturando = False
        self.datos = []
        self.meta = {}
        self.fin = threading.Event()
        self._vivo = True
        self.hilo = threading.Thread(target=self._leer, daemon=True)
        self.hilo.start()

    def _leer(self):
        buf = b""
        while self._vivo:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                break
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self._procesar(linea.decode(errors="ignore").strip())

    def _procesar(self, l):
        if not l:
            return
        if l.startswith("S,"):
            p = l.split(",")
            if len(p) >= 7:
                try:
                    self.estado = dict(estado=int(p[1]), ref=float(p[2]), theta=float(p[3]),
                                       u=float(p[4]), modo=int(p[5]), homed=int(p[6]),
                                       t=time.time())
                except ValueError:
                    pass
            return
        if l.startswith("#"):
            self.mensajes.put(l)
            if l.startswith("# Kp="):                       # parametros del controlador
                for k, v in re.findall(r"(\w+)=([-\d.]+)", l):
                    self.meta[k] = float(v)
            if l.startswith("# fin"):
                self.fin.set()
            if "FALLA" in l:
                print("\n  !!", l)
            return
        if l.startswith("t_s"):
            return
        if self.capturando:
            p = l.split(",")
            if len(p) == 4:
                try:
                    self.datos.append([float(x) for x in p])
                except ValueError:
                    pass

    # ------------------------------------------------------------ comandos
    def enviar(self, cmd):
        self.ser.write((cmd + "\n").encode())
        time.sleep(0.03)

    def esperar_mensaje(self, patron, timeout=10.0, mostrar=True):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                m = self.mensajes.get(timeout=0.2)
            except queue.Empty:
                continue
            if mostrar:
                print("   ", m)
            if re.search(patron, m):
                return m
        return None

    def vaciar_mensajes(self, mostrar=False):
        while not self.mensajes.empty():
            m = self.mensajes.get()
            if mostrar:
                print("   ", m)

    def mostrar_hasta_terminar(self, patron_fin=None, timeout=90.0, silencio=1.5):
        """Muestra los mensajes del Arduino hasta que aparezca 'patron_fin', o hasta
           que el servo deje de estar ocupado (buscando el 0 / calibrando) y pase
           'silencio' segundos sin mensajes. Devuelve la lista de mensajes."""
        vistos = []; t0 = time.time(); ultimo = time.time()
        while time.time() - t0 < timeout:
            try:
                m = self.mensajes.get(timeout=0.2)
                print("   ", m); vistos.append(m); ultimo = time.time()
                if patron_fin and re.search(patron_fin, m):
                    break
                if "FALLA" in m:
                    break
                continue
            except queue.Empty:
                pass
            e = self.estado
            ocupado = e is not None and e["estado"] in (4, 5)
            if not ocupado and patron_fin is None and time.time() - ultimo > silencio:
                break
        return vistos

    def esperar_listo(self, timeout=90.0):
        """Espera a que el servo tenga el 0 y este en lazo cerrado."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.vaciar_mensajes(mostrar=True)
            e = self.estado
            if e and e["homed"] == 1 and e["estado"] == 1 and time.time() - e["t"] < 0.5:
                return True
            if e and e["estado"] == 3:
                return False
            time.sleep(0.2)
        return False

    def ir_a(self, ref, tol=0.3, t_quieto=0.5, timeout=15.0):
        """Envia R<ref> y espera a que el eje quede quieto en la referencia."""
        self.enviar(f"R{ref:g}")
        t0 = time.time(); desde = None
        while time.time() - t0 < timeout:
            e = self.estado
            if e and abs(e["ref"] - ref) < 1e-3 and abs(e["theta"] - ref) < tol:
                desde = desde or time.time()
                if time.time() - desde >= t_quieto:
                    return True
            else:
                desde = None
            time.sleep(0.05)
        return False

    def capturar(self, iniciar, hasta_fin=True, duracion=None, acciones=None):
        """Captura la telemetria a 100 Hz.
           iniciar   : comando que arranca la prueba (T1, A90, L1...)
           hasta_fin : espera el mensaje '# fin' del firmware
           duracion  : o captura durante 'duracion' segundos y luego envia L0
           acciones  : lista [(t, comando)] que se envian durante la captura
        """
        self.datos = []; self.meta = {}; self.fin.clear()
        self.vaciar_mensajes()
        self.capturando = True
        self.enviar(iniciar)
        t0 = time.time()
        pend = sorted(acciones or [])
        try:
            while True:
                ahora = time.time() - t0
                while pend and ahora >= pend[0][0]:
                    self.enviar(pend.pop(0)[1])
                if hasta_fin and self.fin.is_set():
                    break
                if duracion is not None and ahora >= duracion:
                    self.enviar("L0")
                    break
                if ahora > 120:
                    print("   (tiempo maximo de captura)")
                    break
                time.sleep(0.01)
            time.sleep(0.15)
        finally:
            self.capturando = False
        d = np.array(self.datos) if self.datos else np.zeros((0, 4))
        print(f"    {len(d)} muestras capturadas")
        return d, dict(self.meta)

    def cerrar(self):
        self._vivo = False
        try:
            self.enviar("L0")
            self.ser.close()
        except Exception:
            pass


def detectar_puerto():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("arduino", "ch340", "usb serial", "usb-serial", "wch")):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    if ports:
        print("Puertos disponibles:")
        for i, p in enumerate(ports):
            print(f"  {i}: {p.device}  {p.description}")
        return ports[int(input("Numero del puerto del Arduino: "))].device
    sys.exit("No se encontro ningun puerto serie. Conecte el Arduino.")


# =============================================================================
#  Metricas
# =============================================================================
def metricas(d):
    """Metricas de cada escalon de referencia dentro de una captura."""
    if len(d) < 10:
        return []
    t, r, y, u = d.T
    cambios = [0] + [i for i in range(1, len(r)) if abs(r[i] - r[i - 1]) > 1e-6] + [len(r)]
    filas = []
    for a, b in zip(cambios[:-1], cambios[1:]):
        if b - a < 20:
            continue
        r0 = r[a - 1] if a > 0 else y[a]
        rf = r[a]
        paso = rf - r0
        if abs(paso) < 0.5:
            continue
        ts_ = t[a:b] - t[a]; ys = y[a:b]; us = u[a:b]
        s = np.sign(paso)
        avance = (ys - r0) * s                      # recorrido en el sentido del paso
        Mp = max(0.0, (avance.max() - abs(paso)) / abs(paso) * 100)
        try:
            t10 = ts_[np.argmax(avance >= 0.1 * abs(paso))]
            t90 = ts_[np.argmax(avance >= 0.9 * abs(paso))]
            tr = t90 - t10 if avance.max() >= 0.9 * abs(paso) else np.nan
        except ValueError:
            tr = np.nan
        tol = max(0.02 * abs(paso), 0.5)
        fuera = np.where(np.abs(ys - rf) > tol)[0]
        if len(fuera) == 0:
            tset = 0.0
        elif fuera[-1] + 1 < len(ts_):
            tset = ts_[fuera[-1] + 1]
        else:
            tset = np.nan                            # no se establecio
        n_fin = 10                                   # ultimos 0.1 s del tramo
        ess = float(np.mean(ys[-n_fin:] - rf))
        filas.append(dict(t_inicio=round(t[a], 3), r_inicial=round(r0, 2), r_final=round(rf, 2),
                          escalon=round(paso, 2), Mp_pct=round(Mp, 2),
                          tr_s=round(tr, 3) if not np.isnan(tr) else "",
                          ts_s=round(tset, 3) if not np.isnan(tset) else "no",
                          ess_deg=round(ess, 3), u_max_pct=round(float(np.max(np.abs(us))), 1)))
    return filas


def imprimir_metricas(filas, titulo=""):
    if not filas:
        print("    (sin escalones para analizar)")
        return
    if titulo:
        print(f"\n    {titulo}")
    print("    escalon    Mp(%)   tr(s)   ts(s)   ess(deg)  |u|max(%)")
    for f in filas:
        print(f"    {f['escalon']:>7}  {f['Mp_pct']:>6}  {str(f['tr_s']):>6}  {str(f['ts_s']):>6}"
              f"  {f['ess_deg']:>8}  {f['u_max_pct']:>8}")


# =============================================================================
#  Guardado
# =============================================================================
def nueva_carpeta(nombre):
    c = os.path.join("resultados", datetime.now().strftime("%Y-%m-%d_%H%M%S") + "_" + nombre)
    os.makedirs(c, exist_ok=True)
    return c


def nombre_archivo(txt):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", str(txt)).strip("_")


def guardar_datos(carpeta, series, meta=None):
    """series: dict {etiqueta: array Nx4}. Un CSV por serie y un Excel con una hoja por serie."""
    for etq, d in series.items():
        nom = "datos.csv" if len(series) == 1 else f"datos_{nombre_archivo(etq)}.csv"
        with open(os.path.join(carpeta, nom), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "ref_deg", "theta_deg", "u_pct"])
            w.writerows(np.round(d, 4).tolist())
    if openpyxl:
        wb = openpyxl.Workbook(); wb.remove(wb.active)
        for etq, d in series.items():
            ws = wb.create_sheet(nombre_archivo(etq)[:31])
            ws.append(["t (s)", "Referencia (deg)", "Posicion (deg)", "Accion de control (%)"])
            for fila in d.tolist():
                ws.append([round(x, 4) for x in fila])
        if meta:
            ws = wb.create_sheet("parametros")
            for k, v in meta.items():
                ws.append([k, v])
        wb.save(os.path.join(carpeta, "datos.xlsx"))


def guardar_metricas(carpeta, filas_por_serie):
    filas = []
    for etq, fs in filas_por_serie.items():
        for f in fs:
            filas.append({"serie": etq, **f})
    if not filas:
        return
    with open(os.path.join(carpeta, "metricas.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader(); w.writerows(filas)


def guardar_figura(fig, carpeta, nombre="figura"):
    fig.savefig(os.path.join(carpeta, nombre + ".png"))
    fig.savefig(os.path.join(carpeta, nombre + ".pdf"))
    plt.close(fig)
    print(f"    Guardado en: {os.path.abspath(carpeta)}")


# =============================================================================
#  Graficas
# =============================================================================
def figura_respuesta(series, titulo, rel=False):
    """Posicion + referencia (arriba) y accion de control (abajo), eje de tiempo comun.
       series: dict {etiqueta: array Nx4}. rel=True grafica theta - theta(0)."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(ANCHO_COL, 3.4), sharex=True,
                                   gridspec_kw=dict(height_ratios=[2.2, 1], hspace=0.12))
    varias = len(series) > 1
    primera = True
    for i, (etq, d) in enumerate(series.items()):
        if len(d) == 0:
            continue
        t, r, y, u = d.T
        y0 = y[0] if rel else 0.0
        r_ = r - (r[0] if rel else 0.0)
        col = SERIES[i % len(SERIES)]
        if primera:
            ax1.step(t, r_, where="post", color=COL_REF, ls="--", lw=1.0, label="Referencia")
            primera = False
        ax1.plot(t, y - y0, color=col, label=(f"{etq}" if varias else "Posición θ"))
        ax2.plot(t, u, color=col, lw=1.0, label=(f"{etq}" if varias else None))
    ax1.set_ylabel("Posición (°)")
    ax1.set_title(titulo, loc="left")
    ax1.legend(loc="best", ncol=1)
    ax2.set_ylabel("u (%)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylim(-110, 110)
    ax2.set_yticks([-100, -50, 0, 50, 100])
    fig.align_ylabels([ax1, ax2])
    return fig


# =============================================================================
#  Pruebas
# =============================================================================
def preparar(s, modo=2):
    s.enviar(f"M{modo}")
    s.vaciar_mensajes()
    if not s.ir_a(0):
        print("    (el eje no se establecio en 0, se continua igual)")
    time.sleep(0.3)


def prueba_secuencia(s, num=1, modo=2, etiqueta=None, guardar=True):
    nombres = {1: "secuencia T1 (90, -45, 360, 0)", 2: "secuencia T2 (escalones pequeños)"}
    print(f"\n>> {nombres.get(num, 'T')} - modo M{modo}")
    preparar(s, modo)
    d, meta = s.capturar(f"T{num}", hasta_fin=True)
    s.enviar("M2")
    filas = metricas(d)
    imprimir_metricas(filas)
    if guardar:
        c = nueva_carpeta(f"T{num}_M{modo}" + (f"_{etiqueta}" if etiqueta else ""))
        guardar_datos(c, {f"M{modo}": d}, meta)
        guardar_metricas(c, {f"M{modo}": filas})
        tit = f"Seguimiento de referencia, PID (M{modo})" + (f", {etiqueta}" if etiqueta else "")
        guardar_figura(figura_respuesta({f"M{modo}": d}, tit), c)
    return d, meta, filas


def prueba_modos(s, num=1):
    print("\n>> Comparacion de modos: M0 (P), M1 (P + comp. zona muerta), M2 (PID)")
    series, mets, metas = {}, {}, {}
    nombres = {0: "P", 1: "P + comp. ZM", 2: "PID + comp. ZM"}
    for m in (0, 1, 2):
        d, meta, filas = prueba_secuencia(s, num, m, guardar=False)
        series[nombres[m]] = d; mets[nombres[m]] = filas; metas.update(meta)
    c = nueva_carpeta(f"comparacion_modos_T{num}")
    guardar_datos(c, series, metas)
    guardar_metricas(c, mets)
    guardar_figura(figura_respuesta(series, "Efecto de la compensación y de la acción integral"), c)
    # detalle: acercamiento a la referencia en el primer escalon
    fig, ax = plt.subplots(figsize=(ANCHO_COL, 2.2))
    for i, (etq, d) in enumerate(series.items()):
        if len(d) == 0:
            continue
        t, r, y, u = d.T
        k = np.argmax(np.abs(np.diff(r)) > 1e-6) + 1
        rf = r[k]
        sel = (t >= t[k]) & (t <= t[k] + 2.0)
        ax.plot(t[sel] - t[k], y[sel] - rf, color=SERIES[i], label=etq)
    ax.axhline(0, color=COL_REF, ls="--", lw=0.8)
    ax.set_ylim(-3, 3)
    ax.set_xlabel("Tiempo desde el escalón (s)")
    ax.set_ylabel("Error θ − r (°)")
    ax.set_title("Detalle del error final (primer escalón)", loc="left")
    ax.legend()
    guardar_figura(fig, c, "figura_detalle_error")


def prueba_lazo_abierto(s, amp=90.0):
    print(f"\n>> Lazo abierto vs lazo cerrado ({amp:g}°)")
    preparar(s)
    print("    Lazo abierto: pulso de 60 % durante el tiempo calculado con el modelo")
    d_ol, meta = s.capturar(f"A{amp:g}", hasta_fin=True)
    time.sleep(0.5)
    preparar(s)
    print("    Lazo cerrado: PID")
    d_cl, _ = s.capturar("L1", hasta_fin=False, duracion=2.2, acciones=[(0.2, f"R{amp:g}")])
    # alinear tiempos: t = 0 en el inicio del movimiento
    series = {}
    for etq, d in (("Lazo abierto", d_ol), ("Lazo cerrado (PID)", d_cl)):
        if len(d) == 0:
            continue
        d = d.copy()
        if etq.startswith("Lazo cerrado"):
            k = np.argmax(np.abs(np.diff(d[:, 1])) > 1e-6) + 1
            d = d[k - 5:]
        d[:, 0] -= d[0, 0]
        # lazo abierto: recorrido desde donde arranco; lazo cerrado: posicion absoluta
        d[:, 2] -= d[0, 2] if etq == "Lazo abierto" else d[0, 1]
        d[:, 1] = amp
        d[0, 1] = 0.0
        series[etq] = d
    c = nueva_carpeta(f"lazo_abierto_vs_cerrado_{amp:g}")
    guardar_datos(c, series, meta)
    mets = {k: metricas(v) for k, v in series.items()}
    for k, v in series.items():
        if len(v):
            print(f"    {k}: posicion final {v[-1, 2]:.2f}°  (error {v[-1, 2] - amp:+.2f}°)")
    guardar_metricas(c, mets)
    guardar_figura(figura_respuesta(series, f"Posicionamiento a {amp:g}°: lazo abierto vs. cerrado"), c)
    s.enviar("G")


def prueba_perturbacion(s):
    print("\n>> Prueba de perturbacion (roce o carga aplicada a mano)")
    print("    Se registran 12 s: en t = 2 s la referencia pasa a 90° y en t = 7 s vuelve a 0°.")
    print("    Aplique el roce (dedo o trapo sobre el disco) cuando quiera y suelte.")
    input("    Presione Enter para empezar...")
    preparar(s)
    d, meta = s.capturar("L1", hasta_fin=False, duracion=12.0,
                         acciones=[(2.0, "R90"), (7.0, "R0")])
    filas = metricas(d)
    imprimir_metricas(filas)
    c = nueva_carpeta("perturbacion")
    guardar_datos(c, {"perturbacion": d}, meta)
    guardar_metricas(c, {"perturbacion": filas})
    guardar_figura(figura_respuesta({"PID": d}, "Respuesta ante perturbación de carga"), c)


def prueba_robustez(s):
    print("\n>> Robustez: T1 en varias condiciones (alimentacion, carga, inercia)")
    n = int(input("    ¿Cuantas condiciones? (ej. 3): ") or "2")
    series, mets, metas = {}, {}, {}
    for i in range(n):
        etq = input(f"    Nombre de la condicion {i + 1} (ej. 11V, 12V, con disco): ").strip() or f"cond{i + 1}"
        input(f"    Prepare el montaje para '{etq}' y presione Enter...")
        d, meta, filas = prueba_secuencia(s, 1, 2, etiqueta=etq, guardar=False)
        series[etq] = d; mets[etq] = filas; metas.update(meta)
    c = nueva_carpeta("robustez")
    guardar_datos(c, series, metas)
    guardar_metricas(c, mets)
    guardar_figura(figura_respuesta(series, "Robustez del PID ante cambios de la planta"), c)


def prueba_homing(s, n=10):
    print(f"\n>> Repetibilidad del 0 (homing {n} veces)")
    centros, anchos, vlts = [], [], []
    for i in range(n):
        s.ir_a(90 if i % 2 == 0 else -90)
        s.vaciar_mensajes()
        s.enviar("H")
        bp = bn = an = None
        t0 = time.time()
        while time.time() - t0 < 60:
            try:
                m = s.mensajes.get(timeout=0.5)
            except queue.Empty:
                continue
            if m.startswith("# Borde +"):
                bp = float(m.split(":")[1])
            elif m.startswith("# Borde -"):
                bn = float(m.split(":")[1])
            elif m.startswith("# Ancho"):
                an = float(m.split(":")[1])
            elif "0 encontrado" in m or "FALLA" in m:
                break
        if bp is None or bn is None:
            print(f"    {i + 1}: falló")
            continue
        crudo = (bp + bn) / 2
        vueltas = int(round(crudo / 360.0))        # vueltas completas que dio buscando la marca
        delta = crudo - 360.0 * vueltas
        centros.append(delta); anchos.append(an); vlts.append(vueltas)
        print(f"    {i + 1:>2}: borde+ {bp:8.2f}  borde- {bn:8.2f}  centro {delta:+7.3f}°"
              f"  (dio {vueltas} vuelta(s))  ancho {an:6.2f}°")
        s.esperar_listo(timeout=15)
    if len(centros) < 2:
        return
    # La 1a medida parte del 0 del encendido: se descarta.
    # Si la busqueda NO dio la vuelta completa, el centro mide la repetibilidad.
    # Si dio una vuelta, el centro incluye ademas el error de N en 360 deg.
    c_ = np.array(centros[1:]); a_ = np.array(anchos[1:]); v_ = np.array(vlts[1:])
    c0 = c_[v_ == 0]; c1 = c_[v_ != 0] / np.where(v_[v_ != 0] == 0, 1, v_[v_ != 0])
    resumen = []
    if len(c0) >= 2:
        resumen.append(f"Repetibilidad del 0 (sin vuelta completa, n={len(c0)}): media {c0.mean():+.3f}°, "
                       f"σ = {c0.std(ddof=1):.3f}°, rango {np.ptp(c0):.3f}°")
    if len(c1) >= 1:
        resumen.append(f"Con una vuelta completa (n={len(c1)}): {c1.mean():+.3f}° ± "
                       f"{(c1.std(ddof=1) if len(c1) > 1 else 0):.3f}°  (incluye el error de N "
                       f"en 360°; si es > 1° recalibrar con la opcion 10)")
    resumen.append(f"Ancho de la marca: {a_.mean():.2f}° ± {a_.std(ddof=1):.2f}°")
    print()
    for l in resumen:
        print("    " + l)
    c = nueva_carpeta("repetibilidad_homing")
    with open(os.path.join(c, "homing.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["repeticion", "vueltas_busqueda", "centro_deg", "ancho_deg"])
        for i, (ce, vv, an) in enumerate(zip(centros, vlts, anchos), 1):
            w.writerow([i, vv, round(ce, 4), round(an, 4)])
    with open(os.path.join(c, "resumen.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(resumen) + "\n")
    fig, ax = plt.subplots(figsize=(ANCHO_COL, 2.1))
    idx = np.arange(2, len(centros) + 1)
    ax.axhspan(-0.0836 / 2, 0.0836 / 2, color="#e4e3df", label="±½ cuenta del encoder")
    if (v_ == 0).any():
        ax.plot(idx[v_ == 0], c_[v_ == 0], "o", color=SERIES[0], ms=4, label="Sin vuelta completa")
    if (v_ != 0).any():
        ax.plot(idx[v_ != 0], c_[v_ != 0], "s", color=SERIES[1], ms=4, label="Con una vuelta completa")
    ax.set_xlabel("Repetición")
    ax.set_ylabel("Centro de la marca (°)")
    ax.set_title("Repetibilidad del homing", loc="left")
    ax.legend(loc="best")
    guardar_figura(fig, c)


def calibrar_reduccion(s, vueltas=5):
    print(f"\n>> Calibración de la relación de reducción ({vueltas} vueltas, ~{vueltas * 3 + 3} s)")
    s.vaciar_mensajes()
    s.enviar(f"N{vueltas}")
    msgs = s.mostrar_hasta_terminar(patron_fin=r"GEAR_RATIO", timeout=vueltas * 5 + 20)
    cuentas = [int(m.split("=")[1]) for m in msgs if m.startswith("# Vuelta")]
    N = next((float(m.split("=")[1]) for m in msgs if m.startswith("# Relacion de reduccion")), None)
    if N is None:
        print("    No se obtuvo el resultado (revise los mensajes de arriba).")
        return
    c = np.array(cuentas, dtype=float)
    print(f"\n    Cuentas por vuelta: {c.mean():.1f} ± {c.std(ddof=1) if len(c) > 1 else 0:.1f}")
    print(f"    N = {N:.3f}   (antes 97.87)")
    print(f"    -> En el codigo:  const float GEAR_RATIO = {N:.3f}f;   y vuelva a cargarlo")
    carpeta = nueva_carpeta("calibracion_N")
    with open(os.path.join(carpeta, "calibracion.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["vuelta", "cuentas"])
        for i, x in enumerate(cuentas, 1):
            w.writerow([i, x])
    with open(os.path.join(carpeta, "resumen.txt"), "w", encoding="utf-8") as f:
        f.write(f"Cuentas por vuelta: {c.mean():.2f} +- {c.std(ddof=1) if len(c) > 1 else 0:.2f} (n={len(c)})\n")
        f.write(f"N = {N:.3f} (calibracion Corte 2: 97.87)\n")
    print(f"    Guardado en: {os.path.abspath(carpeta)}")
    s.esperar_listo(timeout=10)


def prueba_personalizada(s):
    print("\n>> Prueba personalizada")
    txt = input("    Referencias en grados separadas por coma (ej. 0,90,180,0): ")
    refs = [float(x) for x in txt.replace(" ", "").split(",") if x]
    dt = float(input("    Segundos en cada referencia (ej. 2): ") or "2")
    nombre = input("    Nombre de la prueba: ").strip() or "personalizada"
    preparar(s)
    acc = [(0.5 + i * dt, f"R{r:g}") for i, r in enumerate(refs)]
    d, meta = s.capturar("L1", hasta_fin=False, duracion=0.5 + len(refs) * dt + 0.5, acciones=acc)
    filas = metricas(d)
    imprimir_metricas(filas)
    c = nueva_carpeta(nombre_archivo(nombre))
    guardar_datos(c, {nombre: d}, meta)
    guardar_metricas(c, {nombre: filas})
    guardar_figura(figura_respuesta({nombre: d}, nombre), c)


# =============================================================================
#  Menu
# =============================================================================
MENU = """
==================== PRUEBAS DEL SERVO ====================
  1. Seguimiento de referencia  (T1: 90°, -45°, 360°, 0°)
  2. Escalones pequeños         (T2: 5°, 10°, -10°, 45°)
  3. Comparación P / P+comp / PID  (efecto de zona muerta e integral)
  4. Lazo abierto vs lazo cerrado  (90°)
  5. Perturbación (roce aplicado a mano)
  6. Robustez (varias condiciones: 11 V / 12 V / carga)
  7. Repetibilidad del 0 (homing 10 veces)
  8. Prueba personalizada
  9. Enviar un comando manual al Arduino
 10. Calibrar la relación de reducción N (5 vueltas)
  0. Salir
============================================================"""


def main():
    ap = argparse.ArgumentParser(description="Pruebas del servo PID")
    ap.add_argument("--puerto", help="puerto serie (ej. COM5 o /dev/ttyUSB0)")
    args = ap.parse_args()
    puerto = args.puerto or detectar_puerto()
    print(f"Conectando a {puerto} ...")
    s = Servo(puerto)
    print("Al abrir el puerto el Arduino se reinicia y busca el 0. Esperando...")
    if not s.esperar_listo():
        print("El servo no quedo listo (revise el Monitor de mensajes arriba).")
    else:
        print("Servo listo, con el 0 definido.")
    try:
        while True:
            print(MENU)
            op = input("Opción: ").strip()
            if op == "0":
                break
            elif op == "1": prueba_secuencia(s, 1)
            elif op == "2": prueba_secuencia(s, 2)
            elif op == "3": prueba_modos(s)
            elif op == "4": prueba_lazo_abierto(s)
            elif op == "5": prueba_perturbacion(s)
            elif op == "6": prueba_robustez(s)
            elif op == "7": prueba_homing(s, int(input("    Repeticiones (10): ") or "10"))
            elif op == "8": prueba_personalizada(s)
            elif op == "9":
                s.vaciar_mensajes()
                s.enviar(input("    Comando: ").strip())
                time.sleep(0.3)
                s.mostrar_hasta_terminar()
            elif op == "10": calibrar_reduccion(s)
            s.vaciar_mensajes(mostrar=True)   # nada se descarta sin mostrarlo
    except KeyboardInterrupt:
        print("\nInterrumpido.")
    finally:
        s.cerrar()


if __name__ == "__main__":
    main()
