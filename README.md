# Servo de posición con PID – JGA25-370 + Arduino UNO

Control de posición angular tipo servo para un motor DC con encoder Hall (JGA25-370), usando Arduino UNO y driver L298N. Proyecto grupal de Control de Procesos Industriales (Control II), Universidad Santo Tomás.

## Contenido

```
├── servo_pid_uno/
│   └── servo_pid_uno.ino   Firmware del Arduino UNO
├── panel_servo.py          Programa de operación en el computador
├── pruebas_servo.py        Programa de pruebas y cálculo de métricas
└── resultados/             Datos experimentales de las pruebas finales
    ├── 2026-11-11_154038_T1_M2/                       Seguimiento de referencias (90°, −135°, 405°, −360°)
    ├── 2026-11-11_154115_comparacion_modos_T1/        P vs P + zona muerta vs PID
    ├── 2026-11-11_154139_lazo_abierto_vs_cerrado_90/  Lazo abierto vs lazo cerrado
    ├── 2026-11-11_154218_perturbacion/                Rechazo de perturbación
    ├── 2026-11-11_154319_repetibilidad_homing/        Repetibilidad del cero absoluto
    └── calibracion_N/                                 Calibración de la relación de reducción
```

Cada carpeta de resultados contiene los datos crudos (`.csv`/`.xlsx`), las métricas calculadas (`metricas.csv`) y la figura correspondiente.

## Hardware

| Componente | Descripción |
|---|---|
| Arduino UNO | Microcontrolador (solo ejecuta el control) |
| JGA25-370 | Motor DC 12 V con encoder Hall magnético |
| L298N | Driver de motor (puente H) |
| Fuente 12 V | Alimentación del motor |
| FC-51 | Sensor infrarrojo para el cero absoluto (D4) |

## Modelo y controlador

- Planta: G(s) = K / [s(τs + 1)], con K = 4.09 °/(s·%) y τ = 68.6 ms.
- Periodo de muestreo Ts = 9.984 ms (Timer2), discretización ZOH.
- PID posicional: Kp = 10, Ki = 5, Kd = 0.5, filtro derivativo N = 50, derivada sobre la medición.
- Compensación de zona muerta suavizada (u_dz = 18 %), anti-windup por integración condicional y saturación ±100 %.
- Ventana de "en posición" con histéresis y freno activo.
- Cero absoluto por homing con el FC-51 (repetibilidad σ = 0.06°) y relación de reducción calibrada N = 98.77.
- Seguridades: bloqueo del motor, sentido invertido del encoder y rechazo de movimientos sin cero.

## Resultados (11 V, PID)

| Referencia | Sobreimpulso | Tiempo de establecimiento | Error final |
|---|---|---|---|
| 90° | 0.42 % | 0.39 s | +0.38° |
| −135° | 0.05 % | 0.53 s | −0.06° |
| 405° | 0.13 % | 1.38 s | −0.24° |
| −360° | 0.16 % | 1.23 s | −0.08° |

| Modo (escalón 90°) | Sobreimpulso | Tiempo de establecimiento | Error final |
|---|---|---|---|
| P | 8.98 % | 0.80 s | 1.08° |
| P + zona muerta | 7.23 % | 0.66 s | 0.49° |
| PID + zona muerta | 0.60 % | 0.39 s | 0.42° |

En lazo abierto el error final fue de 53.7°. Ante perturbaciones el sobreimpulso se mantuvo por debajo de 0.64 % y el error final por debajo de 0.29°.

## Protocolo serial (UART 115200)

| Comando | Función |
|---|---|
| `R` | Ir a un ángulo absoluto |
| `C` | Brújula: ir a 0–359° por el camino más corto |
| `J` | Girar ±X grados desde la posición actual |
| `G` | Activar lazo cerrado en la posición actual |
| `H` | Buscar el cero (homing) |
| `S` | Parar |
| `Z` | Fijar el cero en la posición actual |
| `M0` / `M1` / `M2` | Modo P / P + zona muerta / PID |
| `P`, `I`, `D` | Cambiar Kp, Ki, Kd |
| `U` | Cambiar la compensación de zona muerta |
| `T1` / `T2` | Secuencias de prueba |
| `A` | Prueba en lazo abierto |
| `N` | Calibración de la reducción |
| `?` | Consultar estado |

Estado a 10 Hz: `S,estado,ref,theta,u,modo,homed`. Telemetría a 100 Hz: `t,ref,theta,u`.

## Limitaciones

- Error residual de hasta ≈0.4° por la resolución del encoder (0.083°/cuenta) y la ventana de "en posición", que evita el ciclo límite.
- K y τ varían con el nivel de PWM (no linealidad del actuador).

