// ============================================================================
//  SERVO DE POSICION CON PID DISCRETO  -  v3.1
//  Control de Procesos Industriales, Corte 3 - Universidad Santo Tomas
//
//  Arduino UNO dedicado SOLO al lazo de control (Ts = 9.984 ms, Timer2).
//  Motor JGA25-370 (N = 98.77) + encoder Hall x4 (4346 cuentas/vuelta,
//  0.0828 deg/cuenta) + puente H L298N + sensor IR FC-51 para el 0.
//
//  CONEXIONES
//    D2 <- encoder A        D3 <- encoder B        D4 <- OUT del FC-51
//    D9 -> ENA (PWM)        D10 -> IN1             D11 -> IN2
//    D0/D1: UART (USB del PC o placa de interfaz)
//
//  PROTOCOLO (UART 115200 baudios, una orden por linea terminada en '\n')
//    Movimiento (requieren tener el 0):
//      R<deg>   ir a un angulo absoluto (multivuelta)       ej. R90, R-720
//      C<deg>   brujula: ir a 0-359 por el camino mas corto ej. C270
//      J<deg>   mover relativo                              ej. J-15
//      G        habilitar el lazo en la posicion actual
//      T1 / T2  secuencias de prueba (activan la telemetria)
//      A<deg>   prueba en lazo abierto (pulso de 60 %)
//      N<n>     calibrar la relacion de reduccion con n vueltas
//    Otras:
//      H  buscar el 0        S  paro (frena)        Z  cero manual aqui
//      M0 / M1 / M2   control P / P + comp. zona muerta / PID (defecto)
//      P<kp> I<ki> D<kd> U<udz>   cambiar ganancias en caliente
//      L1 / L0  telemetria a 100 Hz      ?  estado      X  diagnostico
//
//  SALIDAS
//    S,estado,ref,theta,u,modo,homed   estado a 10 Hz (siempre)
//        estado: 0 parado 1 lazo cerrado 2 lazo abierto 3 falla
//                4 buscando el 0 5 calibrando
//    t,ref,theta,u                     telemetria a 100 Hz (con L1, T, A)
//    # ...                             mensajes
// ============================================================================

#include <Arduino.h>

// =============================================================================
//  PARAMETROS
// =============================================================================
// ---- planta
const float GEAR_RATIO     = 98.77f;   // calibrado con el FC-51: 98.772 +- 0.009 (40 vueltas)
const float COUNTS_PER_REV = 4.0f * 11.0f * GEAR_RATIO;      // 4346 cuentas/vuelta
const float DEG_PER_COUNT  = 360.0f / COUNTS_PER_REV;        // 0.0828 deg
const float COUNT_PER_DEG  = COUNTS_PER_REV / 360.0f;

// ENC_SIGNO: con este cableado, PWM positivo (IN1 = 1) hacia contar negativo
// (ya visto en el Corte 2). Con -1, u > 0 produce theta creciente. Si se
// cambian cables y aparece "FALLA: sentido invertido", cambiar a +1.
#define ENC_SIGNO (-1)

// ---- muestreo: Timer2 en CTC, prescaler 1024, OCR2A = 155
const float TS = 156.0f * 1024.0f / 16.0e6f;                 // 9.984 ms

// ---- controlador
// Diseno: PD por asignacion de polos sobre G(s) = K/[s(tau s + 1)] y accion
// integral lenta (Ti = Kp/Ki = 2 s) para eliminar el error por friccion.
float Kp       = 10.0f;      // %/deg
float Ki       = 5.0f;       // %/(deg*s)
float Kd       = 0.5f;       // %*s/deg
const float NF = 50.0f;      // filtro derivativo: Tf = Kd/N = 10 ms
float U_DZ     = 18.0f;      // compensacion de zona muerta [%]
float V_RAMPA  = 3.0f;       // la compensacion crece de 0 a U_DZ en |v| = 0..3 %
float BAND_IN  = 2.0f * DEG_PER_COUNT;   // entra "en posicion" con |e| <= 0.17 deg
float BAND_OUT = 0.5f;                   // y sale solo si |e| > 0.5 deg
const float U_MAX = 100.0f;  // saturacion [%]
uint8_t modo   = 2;          // 0 = P, 1 = P + comp. zona muerta, 2 = PID + comp.

// ---- sensor del 0 (FC-51 frente a una llanta con una marca en el eje de salida)
const bool  HOME_ACTIVO_LOW   = true;    // OUT = 0 V sobre la marca (marca blanca)
const float HOME_OFFSET_DEG   = 0.0f;    // angulo del centro de la marca en el compas
const float HOME_VEL_BUSQUEDA = 180.0f;  // deg/s, ida
const float HOME_VEL_MEDIDA   = 180.0f;  // deg/s, regreso (igual: medicion simetrica)
const float HOME_ADELANTO     = 30.0f;   // la referencia no va mas de 30 deg delante del eje
const float HOME_MAX_DEG      = 400.0f;  // sin marca en ~1.1 vueltas -> falla
const bool  HOME_AL_INICIAR   = true;    // busca el 0 cada vez que enciende

// ---- calibracion de la relacion de reduccion (comando N)
const float CAL_VEL = 120.0f;            // deg/s

// ---- prueba en lazo abierto (comando A): modelo del Corte 2 en U = 60 %
const float U_OL = 60.0f, K_OL = 4.128f, TAU_OL = 0.0686f;

// ---- pines
const uint8_t ENC_A = 2, ENC_B = 3, HOME_PIN = 4, PWM_PIN = 9, IN1 = 10, IN2 = 11;

// =============================================================================
//  ESTADO
// =============================================================================
enum Estado : uint8_t { PARADO, LAZO_CERRADO, LAZO_ABIERTO, FALLA, HOMING, CALIBRANDO };
enum FaseHome : uint8_t { H_SALIR, H_BUSCA_POS, H_CRUZAR, H_BUSCA_NEG };

Estado estado = PARADO;
float ref = 0.0f;                  // referencia [deg]
float u = 0.0f;                    // accion de control aplicada [%]
uint32_t k = 0;                    // indice de muestra

// PID
float I = 0.0f, D = 0.0f, yPrev = 0.0f;
bool  enPos = false;

// encoder y reloj de muestreo
volatile long encCount = 0;
volatile long countSample = 0;     // cuentas congeladas en el instante k*Ts
volatile bool tick = false;
volatile uint16_t overruns = 0;    // muestras en que el lazo no alcanzo a correr

// homing
FaseHome fase = H_SALIR;
volatile long homeHitCount = 0;    // cuentas en el flanco del sensor (por interrupcion)
volatile bool homeHit = false;
volatile bool homeArmado = false;
bool  homed = false;               // true = el 0 del compas esta definido
long  bordePos = 0, bordeNeg = 0;
float yHome0 = 0;                  // posicion al iniciar cada fase
uint8_t nFuera = 0;

// calibracion
int   calN = 3, calHits = 0;
float calDir = 1.0f, calRecorrido = 0;
long  calC0 = 0, calLast = 0;
uint8_t calFuera = 0;

// telemetria y secuencias de prueba
bool logOn = false; uint32_t kLog0 = 0;
struct Paso { float t; float r; };
const Paso SEQ1[] = { {0.0, 0}, {0.5, 90}, {2.5, -45}, {4.5, 360}, {7.5, 0}, {9.5, 0} };
const Paso SEQ2[] = { {0.0, 0}, {0.5, 5}, {1.5, 10}, {2.5, -10}, {3.5, 45}, {5.0, 0}, {6.0, 0} };
const Paso* seq = nullptr; uint8_t seqLen = 0, seqIdx = 0; uint32_t kSeq0 = 0;

// lazo abierto
uint32_t kOLfin = 0, kLogFin = 0; float dirOL = 1.0f;

// seguridades
long cntBloq = 0; uint16_t nBloq = 0;

// =============================================================================
//  INTERRUPCIONES
// =============================================================================
inline bool sensorDetecta() {
  bool nivel = (PIND >> 4) & 1;                 // D4
  return HOME_ACTIVO_LOW ? !nivel : nivel;
}

// Encoder x4 con lectura directa del puerto: PIND bit2 = A (D2), bit3 = B (D3)
void isrA() {
  uint8_t p = PIND;
  if (((p >> 2) & 1) == ((p >> 3) & 1)) encCount -= ENC_SIGNO; else encCount += ENC_SIGNO;
}
void isrB() {
  uint8_t p = PIND;
  if (((p >> 2) & 1) == ((p >> 3) & 1)) encCount += ENC_SIGNO; else encCount -= ENC_SIGNO;
}

// Sensor del 0 (PCINT20 = D4): guarda la cuenta exacta en el flanco
ISR(PCINT2_vect) {
  if (homeArmado && !homeHit && sensorDetecta()) { homeHitCount = encCount; homeHit = true; }
}

// Reloj de muestreo: congela la posicion en el instante k*Ts
ISR(TIMER2_COMPA_vect) {
  countSample = encCount;
  if (tick) overruns++;
  tick = true;
}

// =============================================================================
//  ACTUADOR
// =============================================================================
// u = 0 con freno: ENA = 1 e IN1 = IN2 = 0, el L298N cortocircuita el motor
// ("fast motor stop") y la fcem lo frena. Sin freno (ENA = 0) queda en rueda
// libre; solo se usa en la prueba de lazo abierto, igual que en el Corte 2.
void aplicarPWM(float uPct, bool freno = true) {
  uPct = constrain(uPct, -U_MAX, U_MAX);
  uint8_t duty = (uint8_t)(fabs(uPct) * 2.55f + 0.5f);
  if (uPct > 0)      { PORTB |=  _BV(PB2); PORTB &= ~_BV(PB3); }   // IN1 = 1, IN2 = 0
  else if (uPct < 0) { PORTB &= ~_BV(PB2); PORTB |=  _BV(PB3); }   // IN1 = 0, IN2 = 1
  else               { PORTB &= ~(_BV(PB2) | _BV(PB3)); duty = freno ? 255 : 0; }
  analogWrite(PWM_PIN, duty);
}

// =============================================================================
//  LEY DE CONTROL (una vez por periodo)
// =============================================================================
void resetPID(float y) { I = 0; D = 0; yPrev = y; enPos = false; }

float pid(float r, float y) {
  float e = r - y;

  // Derivada sobre la medicion, filtrada (Euler hacia atras):
  // D[k] = a D[k-1] - b (y[k] - y[k-1]),  a = Kd/(Kd + N Ts),  b = Kd N/(Kd + N Ts)
  float a = Kd / (Kd + NF * TS);
  float b = Kd * NF / (Kd + NF * TS);
  D = a * D - b * (y - yPrev);
  yPrev = y;

  if (modo == 0) return constrain(Kp * e, -U_MAX, U_MAX);   // P puro, sin nada mas

  // Ventana "en posicion" con histeresis: al llegar se frena el motor y se
  // reinicia el integrador; solo vuelve a actuar si |e| supera BAND_OUT.
  // Evita el ciclo limite entre friccion estatica, integrador y compensacion.
  if (enPos) {
    if (fabs(e) > BAND_OUT) enPos = false;
    else return 0.0f;
  } else if (fabs(e) <= BAND_IN) {
    enPos = true; I = 0.0f;
    return 0.0f;
  }

  float v = Kp * e + (modo == 2 ? (I + D) : 0.0f);

  // Compensacion de zona muerta suavizada: +-U_DZ para |v| >= V_RAMPA y
  // proporcional cerca de 0, para que el ruido de la derivada no haga
  // saltar u entre +18 % y -18 %.
  float uc = v + U_DZ * constrain(v / V_RAMPA, -1.0f, 1.0f);
  float us = constrain(uc, -U_MAX, U_MAX);

  // Anti-windup: integra solo si no satura o si el error saca de la saturacion
  if (modo == 2 && (us == uc || (e > 0) != (uc > 0))) I += Ki * TS * e;

  return us;
}

// =============================================================================
//  HOMING: busca la marca desde los dos lados y toma su CENTRO como 0
// =============================================================================
// 1) SALIR: si arranca sobre la marca, se aleja.
// 2) BUSCA_POS: avanza a 180 deg/s hasta el borde 1 (capturado por interrupcion).
// 3) CRUZAR: atraviesa la marca y, al salir, invierte la rampa sin detenerse.
// 4) BUSCA_NEG: regresa a 180 deg/s hasta el borde 2.
// 5) 0 = (borde1 + borde2)/2: cancela el ancho del haz, la histeresis del
//    comparador y el juego de los engranajes. Repetibilidad medida: 0.06 deg.
void iniciarHoming(float y) {
  homed = false; homeArmado = false; homeHit = false; nFuera = 0;
  ref = y; yHome0 = y; resetPID(y); seq = nullptr; logOn = false;
  if (sensorDetecta()) fase = H_SALIR;
  else { fase = H_BUSCA_POS; homeArmado = true; }
  estado = HOMING;
  Serial.println(F("# Buscando el 0..."));
}

void fallaHoming(const __FlashStringHelper* msg) {
  homeArmado = false; estado = FALLA;
  Serial.print(F("# FALLA homing: ")); Serial.println(msg);
}

// Devuelve true cuando el 0 quedo definido (y corrige c, y con el desplazamiento)
bool pasoHoming(bool sNow, long& c, float& y) {
  switch (fase) {
    case H_SALIR:
      ref = min(ref + HOME_VEL_BUSQUEDA * TS, y + HOME_ADELANTO);
      if (!sNow && fabs(y - yHome0) > 10.0f) {
        fase = H_BUSCA_POS; yHome0 = y; homeHit = false; homeArmado = true;
      } else if (fabs(y - yHome0) > 90.0f) fallaHoming(F("sensor siempre activo (ajustar potenciometro)"));
      break;

    case H_BUSCA_POS:
      ref = min(ref + HOME_VEL_BUSQUEDA * TS, y + HOME_ADELANTO);
      if (homeHit) {
        homeArmado = false; bordePos = homeHitCount;
        Serial.print(F("# Borde + en [deg]: ")); Serial.println(bordePos * DEG_PER_COUNT, 2);
        yHome0 = y; nFuera = 0; fase = H_CRUZAR;
      } else if (fabs(y - yHome0) > HOME_MAX_DEG) fallaHoming(F("no se encontro la marca"));
      break;

    case H_CRUZAR:
      ref = min(ref + HOME_VEL_BUSQUEDA * TS, y + HOME_ADELANTO);
      if (!sNow) {
        if (++nFuera >= 3) {               // 30 ms fuera de la marca (sin rebote)
          ref = y; yHome0 = y; homeHit = false; homeArmado = true; fase = H_BUSCA_NEG;
        }
      } else nFuera = 0;
      if (fabs(y - yHome0) > 120.0f) fallaHoming(F("no sale de la marca (marca muy ancha?)"));
      break;

    case H_BUSCA_NEG:
      ref = max(ref - HOME_VEL_MEDIDA * TS, y - HOME_ADELANTO);
      if (homeHit) {
        homeArmado = false; bordeNeg = homeHitCount;
        Serial.print(F("# Borde - en [deg]: ")); Serial.println(bordeNeg * DEG_PER_COUNT, 2);
        Serial.print(F("# Ancho de la marca [deg]: "));
        Serial.println((bordeNeg - bordePos) * DEG_PER_COUNT, 2);
        // nuevo sistema de coordenadas: el centro de la marca queda en HOME_OFFSET_DEG
        long centro = (bordePos + bordeNeg) / 2;
        long shift = -centro + (long)(HOME_OFFSET_DEG * COUNT_PER_DEG);
        noInterrupts(); encCount += shift; countSample += shift; c = countSample; interrupts();
        y += shift * DEG_PER_COUNT;
        Serial.println(F("# 0 encontrado (centro de la marca). Yendo a 0"));
        return true;
      } else if (yHome0 - y > 90.0f) fallaHoming(F("la marca no aparecio al regresar"));
      break;
  }
  return false;
}

// =============================================================================
//  CALIBRACION DE N: cuentas entre detecciones consecutivas del mismo borde
// =============================================================================
void pasoCalibracion(bool sNow, float y) {
  ref += calDir * CAL_VEL * TS; calRecorrido += CAL_VEL * TS;
  // rearmar tras 0.2 s (~24 deg) fuera de la marca: el rebote al salir no cuenta
  if (!homeArmado) { if (!sNow) { if (++calFuera > 20) homeArmado = true; } else calFuera = 0; }
  if (homeHit) {
    long hc = homeHitCount; homeHit = false; homeArmado = false; calFuera = 0;
    if (calHits == 0) {
      calC0 = hc;
      Serial.println(F("# Marca inicial detectada"));
    } else {
      Serial.print(F("# Vuelta ")); Serial.print(calHits);
      Serial.print(F(": cuentas = ")); Serial.println(labs(hc - calLast));
    }
    calLast = hc; calHits++;
    if (calHits > calN) {
      float cpr = (float)labs(hc - calC0) / calN;
      Serial.print(F("# Cuentas por vuelta (promedio) = ")); Serial.println(cpr, 2);
      Serial.print(F("# Relacion de reduccion N = ")); Serial.println(cpr / 44.0f, 3);
      Serial.print(F("# Error del N actual en una vuelta [deg] = "));
      Serial.println((cpr - COUNTS_PER_REV) * DEG_PER_COUNT, 2);
      Serial.print(F("# -> poner en el codigo: const float GEAR_RATIO = "));
      Serial.print(cpr / 44.0f, 3); Serial.println(F("f;"));
      ref = y; resetPID(y); estado = LAZO_CERRADO;
      return;
    }
  }
  if (calRecorrido > (calN + 1.5f) * 360.0f) {
    estado = FALLA; Serial.println(F("# FALLA calibracion: marca no detectada"));
  }
}

// =============================================================================
//  COMUNICACION
// =============================================================================
float wrap180(float a) {                       // lleva un angulo a (-180, 180]
  a = fmod(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  if (a <= -180.0f) a += 360.0f;
  return a;
}

void ayuda() {
  Serial.println(F("# R<deg> absoluto | C<deg> brujula | J<deg> relativo | H buscar 0 | S paro"));
  Serial.println(F("# M0 P | M1 P+comp | M2 PID | G habilita | Z cero manual | L1/L0 telemetria"));
  Serial.println(F("# T1/T2 secuencias | A<deg> lazo abierto | N<n> calibrar N | ? estado | X diag"));
}

void estadoTx(float y) {
  Serial.print(F("S,")); Serial.print(estado); Serial.print(',');
  Serial.print(ref, 2); Serial.print(','); Serial.print(y, 2); Serial.print(',');
  Serial.print(u, 1);   Serial.print(','); Serial.print(modo); Serial.print(',');
  Serial.println(homed ? 1 : 0);
}

void iniciarLog() {
  logOn = true; kLog0 = k;
  Serial.print(F("# Kp=")); Serial.print(Kp); Serial.print(F(" Ki=")); Serial.print(Ki);
  Serial.print(F(" Kd=")); Serial.print(Kd); Serial.print(F(" Udz=")); Serial.print(U_DZ);
  Serial.print(F(" modo=")); Serial.print(modo); Serial.print(F(" Ts=")); Serial.println(TS, 6);
  Serial.println(F("t_s,ref_deg,theta_deg,u_pct"));
}

void habilitarSiParado(float y) {
  if (estado == PARADO) { resetPID(y); estado = LAZO_CERRADO; }
}

void procesar(char* c) {
  char cmd = toupper(c[0]);
  float val = atof(c + 1);
  float y = countSample * DEG_PER_COUNT;

  bool esMovimiento = strchr("RCJGTAN", cmd) != nullptr;
  if (esMovimiento && (estado == HOMING || estado == CALIBRANDO)) { Serial.println(F("# Espere: ocupado")); return; }
  if (esMovimiento && !homed) { Serial.println(F("# Sin 0: envie H (o Z para cero manual)")); return; }

  switch (cmd) {
    case 'R': ref = val;                    habilitarSiParado(y); break;
    case 'C': ref = y + wrap180(val - y);   habilitarSiParado(y); break;   // camino mas corto
    case 'J': ref = (estado == PARADO ? y : ref) + val; habilitarSiParado(y); break;
    case 'G': resetPID(y); ref = y; estado = LAZO_CERRADO; break;
    case 'H': iniciarHoming(y); break;
    case 'S': estado = PARADO; seq = nullptr; u = 0; aplicarPWM(0); break;
    case 'Z': noInterrupts(); encCount = 0; countSample = 0; interrupts();
              ref = 0; resetPID(0); homed = true;
              Serial.println(F("# Cero manual fijado")); break;
    case 'M': modo = constrain((int)val, 0, 2); resetPID(y); break;
    case 'P': Kp = val; break;
    case 'I': Ki = val; I = 0; break;
    case 'D': Kd = val; break;
    case 'U': U_DZ = val; break;
    case 'L': if (val > 0) iniciarLog(); else logOn = false; break;
    case 'T':
      seq    = ((int)val == 2) ? SEQ2 : SEQ1;
      seqLen = ((int)val == 2) ? sizeof(SEQ2) / sizeof(Paso) : sizeof(SEQ1) / sizeof(Paso);
      resetPID(y); seqIdx = 0; kSeq0 = k + 1;
      estado = LAZO_CERRADO; iniciarLog(); break;
    case 'A': {                              // pulso de U_OL durante el tiempo del modelo
      dirOL = (val >= 0) ? 1.0f : -1.0f;
      float tOn = fabs(val) / (K_OL * U_OL) + TAU_OL;
      ref = y + val; kOLfin = k + (uint32_t)(tOn / TS + 0.5f);
      kLogFin = kOLfin + (uint32_t)(1.5f / TS);
      estado = LAZO_ABIERTO; iniciarLog(); break; }
    case 'N':
      calN = (int)fabs(val); if (calN < 1) calN = 3;
      calDir = (val < 0) ? -1.0f : 1.0f;
      calHits = 0; calRecorrido = 0; calFuera = 0; homeHit = false; homeArmado = false;
      ref = y; resetPID(y); seq = nullptr; logOn = false; estado = CALIBRANDO;
      Serial.print(F("# Calibrando N con ")); Serial.print(calN); Serial.println(F(" vueltas..."));
      break;
    case '?': estadoTx(y); break;
    case 'X':
      Serial.print(F("# D4=")); Serial.print((PIND >> 4) & 1);
      Serial.print(F(" detecta=")); Serial.print(sensorDetecta() ? 1 : 0);
      Serial.print(F(" estado=")); Serial.print(estado);
      Serial.print(F(" fase=")); Serial.print(fase);
      Serial.print(F(" homed=")); Serial.print(homed ? 1 : 0);
      Serial.print(F(" overruns=")); Serial.println(overruns);
      break;
    default: ayuda();
  }
}

char rx[24]; uint8_t rxN = 0;
void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxN) { rx[rxN] = 0; procesar(rx); rxN = 0; }
    } else if (rxN < sizeof(rx) - 1) rx[rxN++] = ch;
  }
}

// =============================================================================
//  SETUP / LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  aplicarPWM(0);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  cli();                                     // Timer2: CTC, /1024, 100.16 Hz
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  OCR2A  = 155;
  TCNT2  = 0;
  TIMSK2 = _BV(OCIE2A);
  sei();

  pinMode(HOME_PIN, INPUT_PULLUP);           // FC-51 con interrupcion por cambio de pin
  PCMSK2 |= _BV(PCINT20);
  PCICR  |= _BV(PCIE2);

  Serial.println(F("# Servo PID v3.1 listo"));
  ayuda();
  if (HOME_AL_INICIAR) iniciarHoming(0);
}

void loop() {
  leerSerie();
  if (!tick) return;

  noInterrupts(); long c = countSample; tick = false; interrupts();
  float y = c * DEG_PER_COUNT;
  k++;

  // ---- secuencia de prueba
  if (seq && seqIdx < seqLen && (k - kSeq0) * TS >= seq[seqIdx].t) {
    ref = seq[seqIdx].r; seqIdx++;
    if (seqIdx >= seqLen) { seq = nullptr; logOn = false; Serial.println(F("# fin")); }
  }

  // ---- sensor del 0: mensajes y deteccion de respaldo por sondeo
  bool sNow = sensorDetecta();
  static bool sPrev = false;
  bool buscando = (estado == HOMING || estado == CALIBRANDO);
  if (buscando && sNow != sPrev) {
    Serial.print(F("# sensor D4: ")); Serial.println(sNow ? F("MARCA") : F("libre"));
  }
  sPrev = sNow;
  if (buscando && homeArmado && !homeHit && sNow) {
    noInterrupts(); homeHitCount = c; homeHit = true; interrupts();
    Serial.println(F("# (marca tomada por sondeo)"));
  }

  // ---- ley de control segun el estado
  switch (estado) {
    case LAZO_CERRADO:
      u = pid(ref, y);
      break;
    case HOMING:
      if (pasoHoming(sNow, c, y)) { homed = true; ref = 0.0f; resetPID(y); estado = LAZO_CERRADO; }
      u = (estado == FALLA) ? 0.0f : pid(ref, y);
      break;
    case CALIBRANDO:
      pasoCalibracion(sNow, y);
      u = (estado == CALIBRANDO) ? pid(ref, y) : 0.0f;
      break;
    case LAZO_ABIERTO:
      u = (k < kOLfin) ? dirOL * U_OL : 0.0f;
      if (k >= kLogFin) { estado = PARADO; logOn = false; Serial.println(F("# fin")); }
      break;
    default:
      u = 0.0f;
  }

  // ---- seguridad 1: bloqueo mecanico (|u| >= 90 % sin moverse durante 1.5 s)
  if (fabs(u) >= 90.0f && labs(c - cntBloq) < 3) {
    if (++nBloq > (uint16_t)(1.5f / TS)) { estado = FALLA; Serial.println(F("# FALLA: bloqueo")); }
  } else { nBloq = 0; cntBloq = c; }

  // ---- seguridad 2: sentido invertido. La reductora no es reversible: con |u|
  // alto el eje nunca debe girar en contra de u durante 0.3 s.
  static long cAnt = 0; static uint8_t nInv = 0;
  float vel = (c - cAnt) * DEG_PER_COUNT / TS; cAnt = c;
  if (fabs(u) >= 80.0f && vel * u < 0 && fabs(vel) > 30.0f) {
    if (++nInv > 30) { estado = FALLA; nInv = 0;
      Serial.println(F("# FALLA: sentido invertido motor/encoder (cambiar ENC_SIGNO)")); }
  } else nInv = 0;

  if (estado == FALLA) u = 0.0f;
  aplicarPWM(u, estado != LAZO_ABIERTO);

  // ---- salidas: telemetria a 100 Hz (si esta activa) y estado a 10 Hz (siempre)
  if (logOn) {
    Serial.print((k - kLog0) * TS, 3); Serial.print(',');
    Serial.print(ref, 2); Serial.print(','); Serial.print(y, 3); Serial.print(',');
    Serial.println(u, 1);
  }
  if (k % 10 == 0) estadoTx(y);
}
