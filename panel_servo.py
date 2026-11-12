#!/usr/bin/env python3
# =============================================================================
#  PANEL DEL SERVO PID - Control de Procesos Industriales, Corte 3
#  Ventana para manejar el servo desde el PC: angulo en vivo, dial tipo
#  brujula, ordenes de posicion (brujula o relativo), cambio P / PID, buscar el 0, paro y una
#  grafica en tiempo real. Opcionalmente graba la sesion en CSV.
#
#  Instalacion (una vez):  pip install pyserial matplotlib numpy
#  Uso:                    python panel_servo.py
#
#  Atajos: Enter = ir    Esc = PARO    Ctrl+H = buscar el 0    Ctrl+N = modo nocturno
# =============================================================================

import collections
import json
import csv
import math
import os
import queue
import threading
import time
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("Falta pyserial:  pip install pyserial")

BAUD = 115200
VENTANA_S = 10.0                       # segundos visibles en la grafica
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "panel_config.json")

# Paletas: claro y nocturno (mismos papeles, valores ajustados a cada fondo)
TEMAS = {
    "claro": dict(fondo="#fcfcfb", panel="#f4f3f0", campo="#ffffff", texto="#0b0b0b",
                  tenue="#6b6a66", borde="#c3c2b7", rejilla="#e4e3df", boton="#ebeae6",
                  boton_act="#dcdbd6", theta="#2a78d6", theta_act="#1c5cab", ref="#52514e",
                  u="#eb6834", paro="#d03b3b", paro_act="#b02e2e"),
    "oscuro": dict(fondo="#1a1a19", panel="#232322", campo="#2b2b29", texto="#f0efec",
                   tenue="#a3a29a", borde="#4a4a47", rejilla="#383835", boton="#2f2f2d",
                   boton_act="#3c3c39", theta="#3987e5", theta_act="#5598e7", ref="#c3c2b7",
                   u="#d95926", paro="#d03b3b", paro_act="#e66767"),
}
ESTADOS = {0: "PARADO", 1: "LAZO CERRADO", 2: "LAZO ABIERTO", 3: "FALLA",
           4: "BUSCANDO EL 0", 5: "CALIBRANDO"}
MODOS = {0: "P", 1: "P + comp. ZM", 2: "PID"}


# =============================================================================
#  Conexion con el Arduino (hilo lector)
# =============================================================================
class Enlace:
    def __init__(self, puerto):
        self.ser = serial.Serial(puerto, BAUD, timeout=0.1)
        self.cola = queue.Queue()              # ('S', dict) | ('T', tupla) | ('#', texto)
        self.vivo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        buf = b""
        while self.vivo:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                self.cola.put(("#", "# Conexion perdida"))
                break
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                l = linea.decode(errors="ignore").strip()
                if not l:
                    continue
                ahora = time.time()
                if l.startswith("S,"):
                    p = l.split(",")
                    if len(p) >= 7:
                        try:
                            self.cola.put(("S", dict(estado=int(p[1]), ref=float(p[2]),
                                                     theta=float(p[3]), u=float(p[4]),
                                                     modo=int(p[5]), homed=int(p[6]), t=ahora)))
                        except ValueError:
                            pass
                elif l.startswith("#"):
                    self.cola.put(("#", l))
                elif l[0].isdigit() or l[0] == "-":
                    p = l.split(",")
                    if len(p) == 4:
                        try:
                            self.cola.put(("T", (ahora, *map(float, p))))
                        except ValueError:
                            pass

    def enviar(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError):
            pass

    def cerrar(self):
        self.vivo = False
        try:
            self.enviar("L0")
            self.ser.close()
        except Exception:
            pass


def puertos():
    ps = list(serial.tools.list_ports.comports())
    orden = sorted(ps, key=lambda p: not any(k in f"{p.description} {p.manufacturer or ''}".lower()
                                             for k in ("arduino", "ch340", "usb serial", "usb-serial", "wch")))
    return [f"{p.device} - {p.description}" for p in orden]


# =============================================================================
#  Ventana
# =============================================================================
AYUDA_MODOS = {
    "C": "BRÚJULA: escriba hacia dónde apuntar, de 0 a 359. El eje\n"
         "va por el camino MÁS CORTO: de 350° a 10° avanza 20°, no\n"
         "retrocede 340°. Si ya apunta ahí, no se mueve.",
    "J": "RELATIVO: gira la cantidad indicada desde donde está.\n"
         "+90 = un cuarto de vuelta en sentido +, -45 retrocede 45°,\n"
         "+720 = dos vueltas completas. Siempre se mueve.",
}


class Panel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Servo PID - Panel de control")
        self.minsize(1100, 660)
        self.enlace = None
        self.sv = dict(estado=-1, ref=0.0, theta=0.0, u=0.0, modo=2, homed=0, t=0.0)
        self.hist = collections.deque(maxlen=int(VENTANA_S * 110))   # (t_pc, ref, theta, u)
        self.t_ultima_tel = 0.0
        self.grabando = None                     # (archivo, writer, t0)
        self.orden_pend = ""                     # ultima orden enviada (para la grabacion)
        self.nocturno = tk.BooleanVar(value=self._leer_config().get("nocturno", True))
        self.c = TEMAS["oscuro" if self.nocturno.get() else "claro"]
        self.estilo = ttk.Style(self)
        self.estilo.theme_use("clam")
        self._construir()
        self.aplicar_tema()
        self.bind("<Return>", lambda e: self.ir())
        self.bind("<Escape>", lambda e: self.paro())
        self.bind("<Control-h>", lambda e: self.cmd("H"))
        self.bind("<Control-n>", lambda e: (self.nocturno.set(not self.nocturno.get()), self.aplicar_tema()))
        self.protocol("WM_DELETE_WINDOW", self.salir)
        self.after(50, self.ciclo)

    # ------------------------------------------------------------- configuracion guardada
    def _leer_config(self):
        try:
            with open(CONFIG, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _guardar_config(self):
        try:
            with open(CONFIG, "w", encoding="utf-8") as f:
                json.dump({"nocturno": self.nocturno.get()}, f)
        except OSError:
            pass

    # ------------------------------------------------------------- tema (claro / nocturno)
    def aplicar_tema(self):
        c = self.c = TEMAS["oscuro" if self.nocturno.get() else "claro"]
        s = self.estilo
        self.configure(bg=c["fondo"])
        s.configure(".", background=c["fondo"], foreground=c["texto"], font=("Segoe UI", 10),
                    bordercolor=c["borde"], lightcolor=c["fondo"], darkcolor=c["fondo"],
                    troughcolor=c["panel"], focuscolor=c["theta"])
        s.configure("TFrame", background=c["fondo"])
        s.configure("TLabel", background=c["fondo"], foreground=c["texto"])
        s.configure("TLabelframe", background=c["fondo"], bordercolor=c["borde"])
        s.configure("TLabelframe.Label", background=c["fondo"], foreground=c["tenue"],
                    font=("Segoe UI", 9, "bold"))
        s.configure("Grande.TLabel", font=("Consolas", 40, "bold"))
        s.configure("Medio.TLabel", font=("Consolas", 13))
        s.configure("Tenue.TLabel", foreground=c["tenue"])
        s.configure("Ayuda.TLabel", foreground=c["tenue"], font=("Segoe UI", 9))
        s.configure("Estado.TLabel", font=("Segoe UI", 11, "bold"))
        s.configure("TButton", background=c["boton"], foreground=c["texto"], padding=(8, 4),
                    bordercolor=c["borde"], lightcolor=c["boton"], darkcolor=c["boton"])
        s.map("TButton", background=[("active", c["boton_act"])])
        s.configure("Sel.TButton", background=c["theta"], foreground="white",
                    lightcolor=c["theta"], darkcolor=c["theta"])
        s.map("Sel.TButton", background=[("active", c["theta_act"])])
        s.configure("Paro.TButton", font=("Segoe UI", 14, "bold"), foreground="white",
                    background=c["paro"], lightcolor=c["paro"], darkcolor=c["paro"], padding=(10, 10))
        s.map("Paro.TButton", background=[("active", c["paro_act"])])
        for w in ("TRadiobutton", "TCheckbutton"):
            s.configure(w, background=c["fondo"], foreground=c["texto"], indicatorbackground=c["campo"],
                        indicatorforeground=c["theta"])
            s.map(w, background=[("active", c["fondo"])], indicatorbackground=[("selected", c["theta"])])
        s.configure("TEntry", fieldbackground=c["campo"], foreground=c["texto"], insertcolor=c["texto"])
        s.configure("TCombobox", fieldbackground=c["campo"], background=c["boton"], foreground=c["texto"],
                    arrowcolor=c["texto"])
        s.map("TCombobox", fieldbackground=[("readonly", c["campo"])], foreground=[("readonly", c["texto"])],
              selectbackground=[("readonly", c["campo"])], selectforeground=[("readonly", c["texto"])])
        self.option_add("*TCombobox*Listbox.background", c["campo"])
        self.option_add("*TCombobox*Listbox.foreground", c["texto"])
        self.option_add("*TCombobox*Listbox.selectBackground", c["theta"])
        # widgets de tk (no ttk)
        self.tx_msg.configure(bg=c["panel"], fg=c["texto"], insertbackground=c["texto"])
        self.dial.configure(bg=c["fondo"])
        self.bt_tema.configure(text="☀ Modo claro" if self.nocturno.get() else "☾ Modo nocturno")
        # grafica
        self.fig.set_facecolor(c["fondo"])
        for ax in (self.ax1, self.ax2):
            ax.set_facecolor(c["fondo"])
            ax.grid(True, color=c["rejilla"], lw=0.6)
            for sp in ("left", "bottom"):
                ax.spines[sp].set_color(c["borde"])
            ax.tick_params(colors=c["tenue"], labelsize=8)
            ax.yaxis.label.set_color(c["tenue"]); ax.xaxis.label.set_color(c["tenue"])
        self.l_ref.set_color(c["ref"]); self.l_th.set_color(c["theta"]); self.l_u.set_color(c["u"])
        leg = self.ax1.legend(loc="upper left", fontsize=8, frameon=False)
        for t in leg.get_texts():
            t.set_color(c["texto"])
        self.canvas.draw_idle()
        self._guardar_config()

    # ------------------------------------------------------------- widgets
    def _construir(self):
        izq = ttk.Frame(self, padding=10); izq.pack(side="left", fill="y")
        der = ttk.Frame(self, padding=(0, 10, 10, 10)); der.pack(side="left", fill="both", expand=True)

        # --- 1. conexion
        f = ttk.LabelFrame(izq, text="1 · CONEXIÓN", padding=8); f.pack(fill="x")
        self.cb_puerto = ttk.Combobox(f, values=puertos(), width=34, state="readonly")
        if self.cb_puerto["values"]:
            self.cb_puerto.current(0)
        self.cb_puerto.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Button(f, text="↻", width=3, command=self.refrescar_puertos).grid(row=0, column=2, padx=(4, 0))
        self.bt_conectar = ttk.Button(f, text="Conectar", command=self.conectar)
        self.bt_conectar.grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.lb_conexion = ttk.Label(f, text="Desconectado", style="Tenue.TLabel")
        self.lb_conexion.grid(row=1, column=1, columnspan=2, sticky="w", pady=(6, 0), padx=6)

        # --- 2. lectura
        f = ttk.LabelFrame(izq, text="2 · POSICIÓN", padding=8); f.pack(fill="x", pady=8)
        self.lb_theta = ttk.Label(f, text="---.--°", style="Grande.TLabel"); self.lb_theta.pack(anchor="w")
        self.lb_vueltas = ttk.Label(f, text="", style="Ayuda.TLabel"); self.lb_vueltas.pack(anchor="w")
        self.lb_ref = ttk.Label(f, text="Ref    ---", style="Medio.TLabel"); self.lb_ref.pack(anchor="w")
        self.lb_err = ttk.Label(f, text="Error  ---", style="Medio.TLabel"); self.lb_err.pack(anchor="w")
        self.lb_u = ttk.Label(f, text="u      ---", style="Medio.TLabel"); self.lb_u.pack(anchor="w")
        self.lb_estado = ttk.Label(f, text="—", style="Estado.TLabel"); self.lb_estado.pack(anchor="w", pady=(6, 0))

        # --- 3. orden de posicion
        f = ttk.LabelFrame(izq, text="3 · IR A", padding=8); f.pack(fill="x")
        self.modo_pos = tk.StringVar(value="C")
        fr = ttk.Frame(f); fr.pack(fill="x")
        for txt, val in (("Brújula (apuntar)", "C"), ("Relativo (girar)", "J")):
            ttk.Radiobutton(fr, text=txt, value=val, variable=self.modo_pos,
                            command=self.cambio_modo_pos).pack(side="left", padx=(0, 10))
        self.lb_ayuda = ttk.Label(f, text=AYUDA_MODOS["C"], style="Ayuda.TLabel", justify="left")
        self.lb_ayuda.pack(anchor="w", pady=(4, 0))
        self.after_idle(self.cambio_modo_pos)
        fr = ttk.Frame(f); fr.pack(fill="x", pady=6)
        self.en_valor = ttk.Entry(fr, width=10, font=("Consolas", 14)); self.en_valor.pack(side="left")
        ttk.Label(fr, text="°").pack(side="left")
        ttk.Button(fr, text="Ir  ⏎", command=self.ir).pack(side="left", padx=8)
        self.lb_rapidos = ttk.Label(f, text="", style="Ayuda.TLabel"); self.lb_rapidos.pack(anchor="w")
        self.fr_rapidos = ttk.Frame(f); self.fr_rapidos.pack(fill="x")
        ttk.Label(f, text="Empujoncitos (siempre relativos):", style="Ayuda.TLabel").pack(anchor="w", pady=(4, 0))
        fr = ttk.Frame(f); fr.pack(fill="x")
        for g in (-90, -10, -1, 1, 10, 90):
            ttk.Button(fr, text=f"{g:+d}", width=4, command=lambda g=g: self.cmd(f"J{g}")).pack(side="left", padx=1)

        # --- 4. tipo de control
        f = ttk.LabelFrame(izq, text="4 · CONTROL", padding=8); f.pack(fill="x", pady=8)
        fr = ttk.Frame(f); fr.pack(fill="x")
        self.bt_modo = {}
        for m in (2, 0, 1):
            b = ttk.Button(fr, text=MODOS[m], command=lambda m=m: self.cmd(f"M{m}"))
            b.pack(side="left", padx=2, expand=True, fill="x"); self.bt_modo[m] = b

        # --- 5. acciones
        f = ttk.Frame(izq); f.pack(fill="x")
        ttk.Button(f, text="Buscar el 0", command=lambda: self.cmd("H")).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(f, text="Demo (T1)", command=lambda: self.cmd("T1")).pack(side="left", expand=True, fill="x", padx=2)
        self.bt_grabar = ttk.Button(f, text="● Grabar", command=self.grabar)
        self.bt_grabar.pack(side="left", expand=True, fill="x", padx=(2, 0))
        ttk.Button(izq, text="PARO  (Esc)", style="Paro.TButton", command=self.paro).pack(fill="x", pady=8)

        # --- 6. mensajes
        f = ttk.LabelFrame(izq, text="6 · MENSAJES DEL ARDUINO", padding=4); f.pack(fill="both", expand=True)
        self.tx_msg = tk.Text(f, height=5, width=44, font=("Consolas", 8), relief="flat",
                              state="disabled", wrap="none")
        self.tx_msg.pack(fill="both", expand=True)

        # --- derecha: dial, ayuda y grafica
        arriba = ttk.Frame(der); arriba.pack(fill="x")
        self.dial = tk.Canvas(arriba, width=300, height=300, highlightthickness=0)
        self.dial.pack(side="left")
        info = ttk.Frame(arriba, padding=(16, 0)); info.pack(side="left", fill="both", expand=True)
        self.bt_tema = ttk.Button(info, text="", command=lambda: (self.nocturno.set(not self.nocturno.get()),
                                                                   self.aplicar_tema()))
        self.bt_tema.pack(anchor="w")
        self.invertir = tk.BooleanVar(value=False)
        ttk.Checkbutton(info, text="Invertir sentido del dial", variable=self.invertir).pack(anchor="w", pady=(8, 0))
        ttk.Label(info, style="Ayuda.TLabel", justify="left", text=(
            "DIAL: la aguja es la posición actual del eje\n"
            "y el punto en el borde es la referencia.\n"
            "Si la aguja gira al revés que el eje real,\n"
            "marque 'Invertir sentido del dial'.\n\n"
            "GRÁFICA (abajo): últimos 10 s. Arriba, la\n"
            "posición θ y la referencia (punteada);\n"
            "abajo, la acción de control u (% de PWM;\n"
            "el signo es el sentido de giro).\n\n"
            "El 0 es el centro de la marca del FC-51.")).pack(anchor="w", pady=8)

        self.fig = Figure(figsize=(7, 3.6), dpi=100)
        self.ax1 = self.fig.add_subplot(211); self.ax2 = self.fig.add_subplot(212, sharex=self.ax1)
        for ax in (self.ax1, self.ax2):
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        self.l_ref, = self.ax1.plot([], [], ls="--", lw=1.2, label="Referencia")
        self.l_th, = self.ax1.plot([], [], lw=1.6, label="Posición θ")
        self.l_u, = self.ax2.plot([], [], lw=1.2)
        self.ax1.set_ylabel("Posición (°)", fontsize=9)
        self.ax2.set_ylabel("u (%)", fontsize=9); self.ax2.set_xlabel("Tiempo (s)", fontsize=9)
        self.ax2.set_ylim(-110, 110); self.ax2.set_yticks([-100, -50, 0, 50, 100])
        self.ax1.set_xlim(-VENTANA_S, 0)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=der)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=(8, 0))

    # Botones rapidos: cada modo tiene los suyos y SIEMPRE usan el modo elegido
    RAPIDOS = {"C": ("Atajos de brújula (apuntar a):", (0, 45, 90, 180, 270), "{g}°"),
               "J": ("Atajos relativos (girar):", (180, 360, 720), "+{g}")}

    def cambio_modo_pos(self):
        m = self.modo_pos.get()
        self.lb_ayuda.config(text=AYUDA_MODOS[m])
        titulo, valores, fmt = self.RAPIDOS[m]
        self.lb_rapidos.config(text=titulo)
        for w in self.fr_rapidos.winfo_children():
            w.destroy()
        for g in valores:
            ttk.Button(self.fr_rapidos, text=fmt.format(g=g), width=5,
                       command=lambda g=g: self.ordenar(m, g)).pack(side="left", padx=1)

    # ------------------------------------------------------------- conexion
    def refrescar_puertos(self):
        self.cb_puerto["values"] = puertos()
        if self.cb_puerto["values"]:
            self.cb_puerto.current(0)

    def conectar(self):
        if self.enlace:
            self.enlace.cerrar(); self.enlace = None
            self.bt_conectar.config(text="Conectar"); self.lb_conexion.config(text="Desconectado")
            return
        sel = self.cb_puerto.get()
        if not sel:
            messagebox.showwarning("Servo", "No hay puertos. Conecte el Arduino y presione ↻.")
            return
        try:
            self.enlace = Enlace(sel.split(" - ")[0])
        except serial.SerialException as e:
            messagebox.showerror("Servo", f"No se pudo abrir el puerto:\n{e}\n\n"
                                          "¿Está abierto el Monitor Serie o el programa de pruebas?")
            return
        self.bt_conectar.config(text="Desconectar")
        self.lb_conexion.config(text=f"{sel.split(' - ')[0]}: el Arduino se reinicia y busca el 0")
        self.hist.clear()

    def cmd(self, c):
        if not self.enlace:
            self.mensaje("# (sin conexión)")
            return
        self.enlace.enviar(c)
        if c != "L1":
            self.orden_pend = (self.orden_pend + " " + c).strip()   # queda en la grabacion

    # ------------------------------------------------------------- acciones
    def ir(self):
        txt = self.en_valor.get().strip().replace(",", ".")
        try:
            v = float(txt)
        except ValueError:
            return
        self.ordenar(self.modo_pos.get(), v)
        self.en_valor.selection_range(0, "end")

    def ordenar(self, m, v):
        """Envia la orden y anota en los mensajes cuanto va a recorrer el eje."""
        if m == "C":
            v %= 360.0
        sv = self.sv
        if self.enlace and sv["estado"] >= 0:
            th = sv["theta"]
            base = sv["ref"] if sv["estado"] == 1 else th
            if m == "C":
                d = (v - th + 180.0) % 360.0 - 180.0; dest = th + d
                txt = f"Brújula {v:g}°"
            else:
                dest = base + v; txt = f"Relativo {v:+g}°"
            rec = dest - th
            if abs(rec) < 0.5:
                nota = "ya está ahí, no se mueve"
            else:
                vueltas = f" ({abs(rec) / 360:.2f} vueltas)" if abs(rec) >= 360 else ""
                nota = f"recorre {rec:+.1f}°{vueltas} → cuenta {dest:.1f}°"
            self.mensaje(f"→ {txt}: {nota}")
        self.cmd(f"{m}{v:g}")

    def paro(self):
        self.cmd("S")

    def grabar(self):
        if self.grabando:
            self.grabando[0].close(); self.grabando = None
            self.bt_grabar.config(text="● Grabar", style="TButton")
            return
        os.makedirs("sesiones", exist_ok=True)
        nombre = os.path.join("sesiones", datetime.now().strftime("sesion_%Y-%m-%d_%H%M%S.csv"))
        f = open(nombre, "w", newline="")
        w = csv.writer(f); w.writerow(["t_pc_s", "ref_deg", "theta_deg", "u_pct", "orden"])
        self.grabando = (f, w, time.time())
        self.bt_grabar.config(text="■ Detener", style="Sel.TButton")
        self.mensaje(f"# Grabando en {os.path.abspath(nombre)}")

    def mensaje(self, txt):
        self.tx_msg.config(state="normal")
        self.tx_msg.insert("end", txt + "\n")
        lineas = int(self.tx_msg.index("end-1c").split(".")[0])
        if lineas > 200:
            self.tx_msg.delete("1.0", f"{lineas - 200}.0")
        self.tx_msg.see("end")
        self.tx_msg.config(state="disabled")

    def salir(self):
        if self.grabando:
            self.grabando[0].close()
        if self.enlace:
            self.enlace.cerrar()
        self.destroy()

    # ------------------------------------------------------------- dial
    def _punto(self, cx, cy, r, deg):
        a = math.radians(deg)
        s = -math.sin(a) if self.invertir.get() else math.sin(a)
        return cx + r * s, cy - r * math.cos(a)

    def dibujar_dial(self):
        c, k = self.dial, self.c
        c.delete("all")
        cx, cy, r = 150, 150, 120
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=k["borde"], width=2)
        for g in range(0, 360, 10):
            largo = 14 if g % 90 == 0 else (8 if g % 30 == 0 else 4)
            x1, y1 = self._punto(cx, cy, r, g); x2, y2 = self._punto(cx, cy, r - largo, g)
            c.create_line(x1, y1, x2, y2, fill=k["tenue"], width=2 if g % 90 == 0 else 1)
            if g % 30 == 0:
                xt, yt = self._punto(cx, cy, r - 26, g)
                c.create_text(xt, yt, text=str(g), fill=k["tenue"], font=("Segoe UI", 8))
        sv = self.sv
        if sv["estado"] >= 0:
            xr, yr = self._punto(cx, cy, r, sv["ref"])
            c.create_oval(xr - 7, yr - 7, xr + 7, yr + 7, fill=k["ref"], outline=k["fondo"], width=2)
            xa, ya = self._punto(cx, cy, r - 18, sv["theta"])
            c.create_line(cx, cy, xa, ya, fill=k["theta"], width=4, capstyle="round")
        c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=k["theta"], outline="")

    # ------------------------------------------------------------- ciclo de refresco
    def ciclo(self):
        if self.enlace:
            try:
                while True:
                    tipo, dato = self.enlace.cola.get_nowait()
                    if tipo == "S":
                        self.sv = dato
                    elif tipo == "T":
                        tpc, _, r, th, uu = dato
                        self.hist.append((tpc, r, th, uu)); self.t_ultima_tel = tpc
                        if self.grabando:
                            self.grabando[1].writerow([round(tpc - self.grabando[2], 3), r, th, uu,
                                                       self.orden_pend])
                            self.orden_pend = ""
                    elif not dato.startswith(("# R<", "# M0", "# T1", "# Kp=")):
                        self.mensaje(dato)
            except queue.Empty:
                pass
            # mantener la telemetria a 100 Hz (el homing y las secuencias la apagan)
            if time.time() - self.t_ultima_tel > 0.6 and self.sv["estado"] not in (4, 5, -1):
                self.enlace.enviar("L1"); self.t_ultima_tel = time.time()
        self.actualizar_vista()
        self.after(60, self.ciclo)

    def actualizar_vista(self):
        sv, k = self.sv, self.c
        viejo = time.time() - sv.get("t", 0) > 1.0
        if sv["estado"] >= 0:
            self.lb_theta.config(text=f"{sv['theta']:8.2f}°")
            vuelta = math.floor(sv["theta"] / 360.0)
            self.lb_ref.config(text=f"Ref   {sv['ref']:8.2f}°")
            self.lb_vueltas.config(text=f"Apunta a {sv['theta'] % 360:5.1f}°  ·  vuelta {vuelta:+d}")
            self.lb_err.config(text=f"Error {sv['ref'] - sv['theta']:+8.2f}°")
            self.lb_u.config(text=f"u     {sv['u']:+7.1f} %")
        est = "SIN DATOS" if (viejo and self.enlace) else ESTADOS.get(sv["estado"], "—")
        if sv["estado"] == 1 and abs(sv["ref"] - sv["theta"]) < 0.5 and not viejo:
            est = "EN POSICIÓN"
        if sv["estado"] >= 0 and not sv["homed"] and sv["estado"] != 4:
            est += "  ·  sin 0"
        self.lb_estado.config(text=est, foreground=k["paro"] if sv["estado"] == 3 else k["texto"])
        for m, b in self.bt_modo.items():
            b.config(style="Sel.TButton" if m == sv["modo"] else "TButton")
        self.dibujar_dial()
        if self.hist:
            d = np.array(self.hist)
            t = d[:, 0] - time.time()
            self.l_ref.set_data(t, d[:, 1]); self.l_th.set_data(t, d[:, 2]); self.l_u.set_data(t, d[:, 3])
            sel = t > -VENTANA_S
            if sel.any():
                lo = min(d[sel, 1].min(), d[sel, 2].min()); hi = max(d[sel, 1].max(), d[sel, 2].max())
                m = max(5.0, 0.08 * (hi - lo))
                self.ax1.set_ylim(lo - m, hi + m)
            self.ax1.set_xlim(-VENTANA_S, 0)
            self.canvas.draw_idle()


if __name__ == "__main__":
    Panel().mainloop()
