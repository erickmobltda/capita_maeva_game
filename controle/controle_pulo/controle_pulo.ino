// =====================================================================
//  Controle do Pulo v3.2 — ESP32 + MPU6050 (GY-521) como teclado Bluetooth
//
//  Pular   -> ESPAÇO, apertado logo que a criança sai do chão
//  Agachar -> segura SETA PARA BAIXO; solta assim que ela começa a subir
//  BOOT    -> ESPAÇO (teste de pareamento sem o sensor)
//
//  Como funciona:
//   1. Giroscópio + acelerômetro (filtro Mahony) dizem a cada 5 ms onde é
//      "para cima", mesmo com o controle inclinado ou girando.
//   2. Só o movimento VERTICAL conta: balançar para os lados é ignorado.
//   3. PULO    = impulso para cima + corpo "sem peso" (no ar) por ar_ms.
//   4. AGACHAR = já desceu desce_cm e ainda descendo a desce_vel.
//      Se no fim da descida não chegou a agacha_cm, solta (era só um mergulho).
//   5. LEVANTAR = subindo a sobe_vel, ou já subiu metade do que agachou.
//   6. Parado, ele se recalibra sozinho. O comando "sensor" (uma vez só)
//      corrige o erro de fábrica de cada eixo do acelerômetro.
//
//  Serial Monitor (115200, final de linha "New Line"):
//    ajuda | teclas | sensor | cal | ver | desfazer | padrao | log 0/1/2
//    <parametro> <valor>      ex.:  desce_vel 0.15
//
//  Botoes opcionais: D18 -> GND = ESPACO | D19 -> GND = SETA BAIXO (enquanto apertado).
//  Funcionam junto com o movimento.
//
//  Só usa bibliotecas do core oficial da Espressif (arduino-esp32 3.3.8+).
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>
#include <BLE2902.h>

const char* BT_NAME = "Controle Pulo";
const int PIN_SDA = 21, PIN_SCL = 22, PIN_BOOT = 0, PIN_LED = 2;
const int PIN_BTN_PULO = 18;    // botao 1 -> ESPACO   (outra perna, na diagonal, no GND)
const int PIN_BTN_BAIXO = 19;   // botao 2 -> SETA BAIXO enquanto apertado

// Saídas usadas pelo algoritmo (implementadas mais abaixo)
void keySpace(bool on);
void keyDown(bool on);
void saveParams();
void saveSensorCal();

// ======================= [ALGORITMO] =======================

// ---------------- Parâmetros ----------------
struct Param { const char* nome; float padrao; float lo; float hi; const char* ajuda; float val; };
Param prm[] = {
  {"desce_vel",    0.18f, 0.08f, 1.50f,   "m/s para baixo para apertar a seta", 0},
  {"desce_cm",     6.0f,  2.0f,  25.0f,   "cm ja descidos para apertar a seta", 0},
  {"agacha_cm",    9.0f,  3.0f,  40.0f,   "descida minima para MANTER a seta (menos = mergulho, solta)", 0},
  {"sobe_vel",     0.25f, 0.08f, 1.50f,   "m/s para cima para soltar a seta", 0},
  {"pulo_vel",     0.50f, 0.20f, 2.00f,   "m/s para cima no impulso do pulo", 0},
  {"ar_g",         0.35f, 0.15f, 0.80f,   "forca vertical abaixo disso = no ar (g)", 0},
  {"ar_ms",        30.0f, 10.0f, 150.0f,  "ms no ar para confirmar o pulo", 0},
  {"espaco_ms",    120.0f, 40.0f, 400.0f, "ms que o Espaco fica apertado", 0},
  {"pouso_ms",     350.0f, 0.0f, 1500.0f, "ms ignorando agachar depois de pousar", 0},
  {"max_baixo_ms", 5000.0f, 500.0f, 30000.0f, "seguranca: solta a seta depois disso (ms)", 0},
};
enum { DESCE_VEL, DESCE_CM, AGACHA_CM, SOBE_VEL, PULO_VEL, AR_G, AR_MS, ESPACO_MS, POUSO_MS, MAX_BAIXO_MS, NP };
static_assert(sizeof(prm) / sizeof(prm[0]) == NP, "parametros fora de sincronia");
#define P(i)   (prm[i].val)
#define PMS(i) ((unsigned long)prm[i].val)

void setParam(int i, float v) { v = fabsf(v); prm[i].val = constrain(v, prm[i].lo, prm[i].hi); }
void defaultParams() { for (int i = 0; i < NP; i++) prm[i].val = prm[i].padrao; }
void printParams() {
  Serial.println("\n--- Parametros (mude com: <nome> <valor>) ---");
  for (int i = 0; i < NP; i++)
    Serial.printf("  %-13s %7.2f   (padrao %6.2f)  %s\n", prm[i].nome, prm[i].val, prm[i].padrao, prm[i].ajuda);
}

int logMode = 1;   // 0 silencio | 1 eventos | 2 grafico (Serial Plotter)
#define LOG(...) do { if (logMode == 1) { Serial.printf("[%7.2fs] ", millis() / 1000.0f); Serial.printf(__VA_ARGS__); } } while (0)

// ---------------- Estado ----------------
const float GRAV = 9.81f, DEG2RAD = 0.01745329f;
const float KP_NORMAL = 0.8f, KP_FAST = 8.0f;  // ganho do filtro (correcao pelo acelerometro)
float VEL_TAU = 4.0f;                          // s, vazamento suave da velocidade (contra deriva)
const int   STILL_N = 30;                      // 30 amostras = 150 ms parado

enum { JP_GROUND, JP_AIR, JP_LANDED };
enum { CR_UP, CR_DOWN };
enum { CAL_OFF, CAL_STILL, CAL_SQUAT, CAL_JUMP, CAL_END };
int jpState = JP_GROUND, crState = CR_UP, calPhase = CAL_OFF;

float accOff[3] = {0, 0, 0}, accScl[3] = {1, 1, 1};   // calibracao de fabrica do acelerometro
bool  accCalOk = false;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // orientacao (sensor -> mundo)
float gBias[3] = {0, 0, 0};             // giroscopio parado (graus/s)
float g0 = 1.0f;                        // quanto o acelerometro mede parado (g)
float fv = 1.0f;                        // forca vertical / g0: 1 parado, 0 no ar
float vel = 0, pos = 0;                 // velocidade (m/s) e altura (m) verticais
float pTop = 0, pBottom = 0;            // onde a descida/subida atual comecou
bool  still = false;
int   stillCnt = 0, quietCnt = 0;
float lastDev = 0, lastGyro = 0;
float agit = 0;                         // agitacao lateral (g): sacudir para os lados nao conta
const float AGIT_MAX = 0.6f;            // acima disso nao aperta nada
unsigned long fastUntil = 0;

// ---------------- Orientacao e movimento vertical ----------------
void sensorSetFromStill(float ax, float ay, float az, float gx, float gy, float gz, unsigned long now) {
  // ax..az ja calibrados; gx..gz crus
  gBias[0] = gx; gBias[1] = gy; gBias[2] = gz;
  float n = sqrtf(ax * ax + ay * ay + az * az);
  g0 = (n > 0.5f && n < 1.5f) ? n : 1.0f;
  float roll = atan2f(ay, az), pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
  vel = pos = pTop = pBottom = 0;
  fastUntil = now + 1000;
}

void imuStep(float ax, float ay, float az, float gx, float gy, float gz, float dt, unsigned long now) {
  float dgx = gx - gBias[0], dgy = gy - gBias[1], dgz = gz - gBias[2];
  float wx = dgx * DEG2RAD, wy = dgy * DEG2RAD, wz = dgz * DEG2RAD;
  float an = sqrtf(ax * ax + ay * ay + az * az);
  float dev = fabsf(an / g0 - 1.0f);
  float gm = sqrtf(dgx * dgx + dgy * dgy + dgz * dgz);

  // O acelerometro so corrige a orientacao depois de 50 ms medindo ~1 g
  // (durante movimento ele "mente": mede gravidade + aceleracao)
  quietCnt = (dev < 0.06f && gm < 60.0f) ? quietCnt + 1 : 0;
  float trust = quietCnt < 10 ? 0.0f : (dev < 0.03f ? 1.0f : (0.06f - dev) / 0.03f);
  float kp = (now < fastUntil ? KP_FAST : KP_NORMAL) * trust;
  if (kp > 0 && an > 0.2f) {
    float vx = 2 * (q1 * q3 - q0 * q2), vy = 2 * (q0 * q1 + q2 * q3), vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float nx = ax / an, ny = ay / an, nz = az / an;
    wx += kp * (ny * vz - nz * vy);
    wy += kp * (nz * vx - nx * vz);
    wz += kp * (nx * vy - ny * vx);
  }
  float hx = 0.5f * dt * wx, hy = 0.5f * dt * wy, hz = 0.5f * dt * wz;
  float a0 = q0, a1 = q1, a2 = q2;
  q0 += -a1 * hx - a2 * hy - q3 * hz;
  q1 +=  a0 * hx + a2 * hz - q3 * hy;
  q2 +=  a0 * hy - a1 * hz + q3 * hx;
  q3 +=  a0 * hz + a1 * hy - a2 * hx;
  float qn = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= qn; q1 *= qn; q2 *= qn; q3 *= qn;

  // "Para cima" no referencial do sensor -> forca vertical
  float ux = 2 * (q1 * q3 - q0 * q2), uy = 2 * (q0 * q1 + q2 * q3), uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  fv = (ax * ux + ay * uy + az * uz) / g0;
  float h2 = an * an / (g0 * g0) - fv * fv;                 // forca horizontal^2
  agit = fmaxf(h2 > 0 ? sqrtf(h2) : 0, agit * 0.985f);      // pico que decai em ~0,3 s

  // Parado? Zera a velocidade e recalibra sozinho (devagar)
  lastDev = dev; lastGyro = gm;
  stillCnt = (dev < 0.04f && gm < 6.0f) ? stillCnt + 1 : 0;
  still = stillCnt >= STILL_N;
  if (still) {
    g0 += 0.005f * (an - g0);
    gBias[0] += 0.005f * dgx; gBias[1] += 0.005f * dgy; gBias[2] += 0.005f * dgz;
  }

  float aLin = constrain(fv - 1.0f, -3.0f, 3.0f) * GRAV;   // m/s2, + = para cima
  vel += aLin * dt;
  if (jpState == JP_GROUND) {
    vel -= vel * dt / VEL_TAU;
    if (still && fabsf(vel) < 0.25f) vel *= 0.85f;
  }
  pos += vel * dt;
}

void resetMotion() {
  vel = 0; pos = 0; pTop = 0; pBottom = 0;
}

// ---------------- Calibracao: registro (usado pelos detectores) ----------------
const int MAXR = 16;
struct Rec { float a, b, c, t; };   // valores + horario (ms)
Rec recDesc[CAL_END][MAXR], recRise[CAL_END][MAXR], recFly[CAL_END][MAXR];
int nDesc[CAL_END], nRise[CAL_END], nFly[CAL_END];
bool calRec = false;

void addRec(Rec (&arr)[CAL_END][MAXR], int (&n)[CAL_END], float a, float b, float c, unsigned long t) {
  if (calRec && n[calPhase] < MAXR) { arr[calPhase][n[calPhase]] = {a, b, c, (float)t}; n[calPhase]++; }
}

// ---------------- Pulo ----------------
float vPeak = 0; unsigned long vPeakAt = 0;
unsigned long lowSince = 0, takeoffAt = 0, landedAt = 0, lastJump = 0, spaceSince = 0, riseGuardUntil = 0;
bool spaceByJump = false;
// episodio "leve" (forca vertical < 0.7 g), para log e calibracao
bool flEp = false, flFired = false; unsigned long flStart = 0; float flMin = 9, flTake = 0, flBest = 0;

void onFlightEnd(unsigned long now) {
  float dur = (float)(now - flStart);
  addRec(recFly, nFly, dur, flMin, flTake, flStart);
  if (flFired) {
    float t = dur / 1000.0f;
    LOG("  pousou   | %.0f ms no ar (~%.0f cm)\n", dur, GRAV * t * t / 8.0f * 100.0f);
  } else if (calPhase == CAL_OFF && (dur >= 60 || flTake >= 0.8f * P(PULO_VEL))) {
    LOG("  quase-pulo | impulso %.2f m/s (pulo_vel %.2f) | forca min %.2f g (ar_g %.2f) | %.0f ms abaixo de ar_g (ar_ms %.0f)\n",
        flTake, P(PULO_VEL), flMin, P(AR_G), flBest, P(AR_MS));
  }
}

void jumpStep(unsigned long now) {
  // maior velocidade para cima nos ultimos 300 ms = impulso
  if (vel > vPeak || now - vPeakAt > 300) { vPeak = vel; vPeakAt = now; }

  if (fv < 0.70f) {
    if (!flEp) { flEp = true; flFired = false; flStart = now; flMin = fv; flTake = vPeak; flBest = 0; lowSince = 0; }
    flMin = fminf(flMin, fv);
    if (fv < P(AR_G)) { if (!lowSince) lowSince = now; flBest = fmaxf(flBest, (float)(now - lowSince)); }
    else lowSince = 0;
  } else if (flEp && (jpState != JP_AIR || now - takeoffAt > 60)) {
    flEp = false; onFlightEnd(now);
  }

  if (jpState == JP_GROUND) {
    // logo depois de levantar de uma agachada, exige mais tempo no ar
    // (levantar rapido tambem deixa o corpo "leve" por um instante)
    unsigned long need = PMS(AR_MS) * (now < riseGuardUntil ? 2 : 1);
    if (flEp && !flFired && lowSince && now - lowSince >= need &&
        flTake >= P(PULO_VEL) && now - lastJump >= 250 && agit < 2.5f * AGIT_MAX) {
      flFired = true;
      jpState = JP_AIR; takeoffAt = lowSince; lastJump = now;
      LOG("PULO       | impulso %.2f m/s | %lu ms apos sair do chao\n", flTake, now - lowSince);
      if (crState == CR_DOWN) { crState = CR_UP; keyDown(false); }
      keySpace(true); spaceByJump = true; spaceSince = now;
    }
  } else if (jpState == JP_AIR) {
    if ((fv > 0.9f && now - takeoffAt > 60) || now - takeoffAt > 800) { jpState = JP_LANDED; landedAt = now; }
  } else if (jpState == JP_LANDED) {
    if (now - landedAt >= PMS(POUSO_MS)) { jpState = JP_GROUND; resetMotion(); }
  }

  if (spaceByJump && now - spaceSince >= PMS(ESPACO_MS)) { keySpace(false); spaceByJump = false; }
}

// ---------------- Agachar ----------------
int crCnt = 0; unsigned long crSince = 0; bool crDescentDone = false; float crMaxDepth = 0;
bool dEp = false, dEpPressed = false; float dEpMinV = 0, dEpDepth = 0;
bool uEp = false; float uEpMaxV = 0, uEpH = 0;

void releaseDown(const char* why, unsigned long now) {
  LOG("SOBE       | %s\n", why);
  crState = CR_UP; crCnt = 0; keyDown(false);
  if (now - crSince >= 300) riseGuardUntil = now + 500;   // so depois de agachada de verdade
}

void crouchStep(unsigned long now) {
  if (jpState != JP_GROUND) { dEp = uEp = false; crCnt = 0; return; }   // no ar / pousando

  if (vel >= -0.02f) pTop = pos;
  if (vel <= 0.02f) pBottom = pos;
  float depth = pTop - pos;     // m descidos nesta descida
  float rise = pos - pBottom;   // m subidos nesta subida

  // episodios (para log e calibracao)
  if (!dEp && vel < -0.06f) { dEp = true; dEpMinV = vel; dEpDepth = 0; dEpPressed = (crState == CR_DOWN); }
  if (dEp) {
    dEpMinV = fminf(dEpMinV, vel); dEpDepth = fmaxf(dEpDepth, depth);
    if (vel >= -0.02f) {
      dEp = false;
      addRec(recDesc, nDesc, -dEpMinV, dEpDepth, 0, now);
      if (!dEpPressed && calPhase == CAL_OFF && dEpDepth * 100 >= P(AGACHA_CM))
        LOG("  quase-baixo | desceu %.0f cm, mas so a %.2f m/s (desce_vel %.2f)\n", dEpDepth * 100, -dEpMinV, P(DESCE_VEL));
    }
  }
  if (!uEp && vel > 0.06f) { uEp = true; uEpMaxV = vel; uEpH = 0; }
  if (uEp) {
    uEpMaxV = fmaxf(uEpMaxV, vel); uEpH = fmaxf(uEpH, rise);
    if (vel <= 0.02f) { uEp = false; addRec(recRise, nRise, uEpMaxV, uEpH, 0, now); }
  }

  if (crState == CR_UP) {
    if (vel < -P(DESCE_VEL) && depth * 100 >= P(DESCE_CM) && agit < AGIT_MAX) {
      if (++crCnt >= 3) {                       // 15 ms de confirmacao
        crState = CR_DOWN; crCnt = 0; crSince = now; crDescentDone = false; crMaxDepth = depth;
        dEpPressed = true;
        LOG("BAIXO      | descendo %.2f m/s, ja %.0f cm\n", -vel, depth * 100);
        keyDown(true);
      }
    } else crCnt = 0;
    return;
  }

  // CR_DOWN
  if (!crDescentDone) {
    crMaxDepth = fmaxf(crMaxDepth, depth);
    if (vel >= -0.03f) {
      crDescentDone = true;
      if (crMaxDepth * 100 < P(AGACHA_CM)) {
        LOG("  mergulho | so desceu %.0f cm (agacha_cm %.0f): solta a seta\n", crMaxDepth * 100, P(AGACHA_CM));
        crState = CR_UP; crCnt = 0; keyDown(false);
        return;
      }
      LOG("  agachado | desceu %.0f cm (seta continua apertada)\n", crMaxDepth * 100);
    }
  }
  if (vel > P(SOBE_VEL) && agit < AGIT_MAX) {
    if (++crCnt >= 3) { char b[48]; snprintf(b, sizeof(b), "subindo %.2f m/s", vel); releaseDown(b, now); return; }
  } else crCnt = 0;
  if (crDescentDone && rise >= fmaxf(0.08f, 0.5f * crMaxDepth) && agit < AGIT_MAX) {
    char b[48]; snprintf(b, sizeof(b), "ja subiu %.0f cm", rise * 100); releaseDown(b, now); return;
  }
  if (now - crSince > PMS(MAX_BAIXO_MS)) releaseDown("tempo maximo", now);
}

// ---------------- Calibracao do sensor (uma vez): 6 posicoes paradas ----------------
const int SC_NEED = 6;
bool scActive = false, scWaitMove = false, scRefOk = false;
int scCount = 0, scN = 0;
float scVec[SC_NEED][3], scRef[6];
double scSum[3];
unsigned long scStart = 0;

bool solveLinear(double* A, double* b, int n) {   // eliminacao de Gauss com pivoteamento
  for (int c = 0; c < n; c++) {
    int p = c;
    for (int r = c + 1; r < n; r++) if (fabs(A[r * n + c]) > fabs(A[p * n + c])) p = r;
    if (fabs(A[p * n + c]) < 1e-12) return false;
    if (p != c) { for (int k = 0; k < n; k++) { double t = A[c * n + k]; A[c * n + k] = A[p * n + k]; A[p * n + k] = t; } double t = b[c]; b[c] = b[p]; b[p] = t; }
    for (int r = 0; r < n; r++) {
      if (r == c) continue;
      double f = A[r * n + c] / A[c * n + c];
      for (int k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
      b[r] -= f * b[c];
    }
  }
  for (int c = 0; c < n; c++) b[c] /= A[c * n + c];
  return true;
}

// Elipsoide alinhado aos eixos: offset e escala por eixo
bool fitEllipsoid(float v[][3], int n, float* off, float* scl) {
  double A[36] = {0}, b[6] = {0};
  for (int i = 0; i < n; i++) {
    double x = v[i][0], y = v[i][1], z = v[i][2];
    double r[6] = {x * x, y * y, z * z, x, y, z};
    for (int j = 0; j < 6; j++) { b[j] += r[j]; for (int k = 0; k < 6; k++) A[j * 6 + k] += r[j] * r[k]; }
  }
  if (!solveLinear(A, b, 6)) return false;
  if (b[0] <= 0 || b[1] <= 0 || b[2] <= 0) return false;
  double o[3], K = 1;
  for (int i = 0; i < 3; i++) { o[i] = -b[3 + i] / (2 * b[i]); K += b[i] * o[i] * o[i]; }
  for (int i = 0; i < 3; i++) {
    off[i] = (float)o[i];
    scl[i] = (float)sqrt(K / b[i]);
    if (fabsf(off[i]) > 0.3f || scl[i] < 0.8f || scl[i] > 1.2f) return false;
  }
  return true;
}

// Esfera: so offset (precisa de menos variedade de posicoes)
bool fitSphere(float v[][3], int n, float* off, float* scl) {
  double A[16] = {0}, b[4] = {0};
  for (int i = 0; i < n; i++) {
    double x = v[i][0], y = v[i][1], z = v[i][2];
    double r[4] = {2 * x, 2 * y, 2 * z, 1}, t = x * x + y * y + z * z;
    for (int j = 0; j < 4; j++) { b[j] += r[j] * t; for (int k = 0; k < 4; k++) A[j * 4 + k] += r[j] * r[k]; }
  }
  if (!solveLinear(A, b, 4)) return false;
  double rad2 = b[3] + b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
  if (rad2 <= 0) return false;
  float rad = (float)sqrt(rad2);
  for (int i = 0; i < 3; i++) {
    off[i] = (float)b[i]; scl[i] = rad;
    if (fabsf(off[i]) > 0.3f || rad < 0.8f || rad > 1.2f) return false;
  }
  return true;
}

void sensorCalStart(unsigned long now) {
  if (spaceByJump) { keySpace(false); spaceByJump = false; }
  if (crState == CR_DOWN) { crState = CR_UP; keyDown(false); }
  scActive = true; scWaitMove = false; scRefOk = false; scCount = 0; scN = 0; scStart = now;
  Serial.println("\n######## CALIBRACAO DO SENSOR (so precisa fazer uma vez) ########");
  Serial.println("Deixe o controle PARADO em 6 posicoes diferentes, como as faces de um dado:");
  Serial.println("em pe, de cabeca para baixo, deitado de frente, de costas, de um lado e do outro.");
  Serial.println("Apoie na mesa ou no sofa, 2 s em cada. A cada posicao aceita aparece 'ok'.");
}

void sensorCalFinish() {
  float off[3], scl[3];
  bool ok = fitEllipsoid(scVec, scCount, off, scl);
  const char* tipo = "offset e escala";
  if (!ok) { ok = fitSphere(scVec, scCount, off, scl); tipo = "so offset"; }
  scActive = false;
  if (!ok) {
    Serial.println("Nao consegui calcular: as posicoes ficaram parecidas demais. Rode 'sensor' de novo variando mais.");
    return;
  }
  float err = 0;
  for (int i = 0; i < scCount; i++) {
    float m = 0;
    for (int k = 0; k < 3; k++) { float c = (scVec[i][k] - off[k]) / scl[k]; m += c * c; }
    err = fmaxf(err, fabsf(sqrtf(m) - 1));
  }
  for (int k = 0; k < 3; k++) { accOff[k] = off[k]; accScl[k] = scl[k]; }
  accCalOk = true;
  saveSensorCal();
  Serial.printf("Pronto (%s). Offset X %+.3f Y %+.3f Z %+.3f g | escala X %.3f Y %.3f Z %.3f | erro max %.3f g\n",
                tipo, off[0], off[1], off[2], scl[0], scl[1], scl[2], err);
  Serial.println("Salvo na placa. Agora segure o controle parado 1 s para ele se ajustar.");
  stillCnt = 0; resetMotion(); crState = CR_UP; jpState = JP_GROUND;
  g0 = 1.0f; fastUntil = 0xFFFFFFFF;   // reinicia a orientacao quando ficar parado
}

void sensorCalSample(float ax, float ay, float az, float gx, float gy, float gz, unsigned long now) {
  float s[6] = {ax, ay, az, gx, gy, gz};
  if (now - scStart > 180000) { scActive = false; Serial.println("Tempo esgotado. Rode 'sensor' de novo."); return; }
  if (!scRefOk) {                                // primeira amostra parada vira referencia
    for (int i = 0; i < 6; i++) scRef[i] = s[i];
    scRefOk = true; scN = 0; scSum[0] = scSum[1] = scSum[2] = 0;
    return;
  }
  bool moved = false;
  for (int i = 0; i < 3; i++) if (fabsf(s[i] - scRef[i]) > 0.03f) moved = true;
  for (int i = 3; i < 6; i++) if (fabsf(s[i] - scRef[i]) > 4.0f) moved = true;
  if (moved) { scRefOk = false; scWaitMove = false; return; }
  if (scWaitMove) return;                        // ja registrou esta posicao: espera mexer
  scSum[0] += ax; scSum[1] += ay; scSum[2] += az; scN++;
  if (scN < 300) return;                         // 1,5 s parado
  float m[3] = {(float)(scSum[0] / scN), (float)(scSum[1] / scN), (float)(scSum[2] / scN)};
  float mn = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  float best = -2;
  for (int i = 0; i < scCount; i++) {
    float vn = sqrtf(scVec[i][0] * scVec[i][0] + scVec[i][1] * scVec[i][1] + scVec[i][2] * scVec[i][2]);
    best = fmaxf(best, (m[0] * scVec[i][0] + m[1] * scVec[i][1] + m[2] * scVec[i][2]) / (mn * vn));
  }
  scWaitMove = true;
  if (best > 0.82f) { Serial.println("  essa posicao ja foi, vire para outro lado"); return; }   // < 35 graus
  for (int k = 0; k < 3; k++) scVec[scCount][k] = m[k];
  scCount++;
  Serial.printf("  posicao %d de %d ok\n", scCount, SC_NEED);
  if (scCount >= SC_NEED) sensorCalFinish();
}

// ---------------- Calibracao dos movimentos ----------------
const unsigned long CAL_DUR[CAL_END] = {0, 2000, 15000, 12000};
const char* CAL_MSG[CAL_END] = {"", "Segure o controle PARADO, do jeito que vai jogar",
                                "AGACHE e LEVANTE 3 vezes (agache de verdade)",
                                "PULE 3 vezes, com uma pausa entre os pulos"};
unsigned long calT0 = 0; int calCount = -1;
float stMaxVel = 0, stMaxDev = 0, stMaxGyro = 0; double stSumA = 0, stSumG[3] = {0, 0, 0}; long stN = 0;
float undoVals[NP]; bool hasUndo = false;

float medianOf(const float* a, int n) {
  if (n <= 0) return 0;
  float t[MAXR];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) { float k = t[i]; int j = i - 1; while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; } t[j + 1] = k; }
  return t[n / 2];
}
float minOf(const float* a, int n) { float m = 1e9f; for (int i = 0; i < n; i++) m = fminf(m, a[i]); return m; }
float maxOf(const float* a, int n) { float m = -1e9f; for (int i = 0; i < n; i++) m = fmaxf(m, a[i]); return m; }

void calStartPhase(unsigned long now) {
  calT0 = now; calCount = -1; calRec = false;
  Serial.printf("\n[%d/3] %s\n", calPhase, CAL_MSG[calPhase]);
}

void calStart(unsigned long now) {
  if (spaceByJump) { keySpace(false); spaceByJump = false; }
  if (crState == CR_DOWN) { crState = CR_UP; keyDown(false); }
  for (int p = 0; p < CAL_END; p++) { nDesc[p] = nRise[p] = nFly[p] = 0; }
  stMaxVel = stMaxDev = stMaxGyro = 0; stSumA = 0; stSumG[0] = stSumG[1] = stSumG[2] = 0; stN = 0;
  Serial.println("\n######## CALIBRACAO DOS MOVIMENTOS (3 etapas, as teclas ficam desligadas) ########");
  if (!accCalOk) Serial.println("Dica: rode 'sensor' antes (uma vez so). Melhora muito a precisao.");
  calPhase = CAL_STILL; calStartPhase(now);
}

void calStillSample(float ax, float ay, float az, float gx, float gy, float gz) {
  stSumA += sqrtf(ax * ax + ay * ay + az * az);
  stSumG[0] += gx; stSumG[1] += gy; stSumG[2] += gz; stN++;
  stMaxVel = fmaxf(stMaxVel, fabsf(vel)); stMaxDev = fmaxf(stMaxDev, lastDev); stMaxGyro = fmaxf(stMaxGyro, lastGyro);
}

void calFinishStill() {
  if (stN > 100 && stMaxGyro < 15 && stMaxDev < 0.10f) {
    g0 = stSumA / stN;
    for (int i = 0; i < 3; i++) gBias[i] = stSumG[i] / stN;
    Serial.printf("  Parado ok: mede %.3f g\n", g0);
  } else {
    Serial.println("  O controle mexeu: mantive o ajuste automatico.");
  }
}

void calReport() {
  Serial.println("\n==================== RESULTADO ====================");
  float sqV[MAXR], sqD[MAXR], ruV[MAXR], jT[MAXR], jF[MAXR], jV[MAXR], dipD[MAXR];
  int ns = 0, nr = 0, nj = 0, nd = 0;
  for (int i = 0; i < nDesc[CAL_SQUAT]; i++)
    if (recDesc[CAL_SQUAT][i].b >= 0.08f) { sqV[ns] = recDesc[CAL_SQUAT][i].a; sqD[ns] = recDesc[CAL_SQUAT][i].b; ns++; }
  for (int i = 0; i < nRise[CAL_SQUAT]; i++)
    if (recRise[CAL_SQUAT][i].b >= 0.08f) ruV[nr++] = recRise[CAL_SQUAT][i].a;
  float jS[MAXR];   // inicio de cada pulo
  for (int i = 0; i < nFly[CAL_JUMP]; i++) {          // pulo = ficou leve 60 ms+ depois de impulso para cima
    Rec& r = recFly[CAL_JUMP][i];
    if (r.a >= 60 && r.c >= 0.30f) { jT[nj] = r.a; jF[nj] = r.b; jV[nj] = r.c; jS[nj] = r.t; nj++; }
  }
  for (int i = 0; i < nDesc[CAL_JUMP]; i++) {         // mergulho = descida que termina ate 600 ms antes de um pulo
    Rec& r = recDesc[CAL_JUMP][i];
    bool before = false;
    for (int k = 0; k < nj; k++) if (jS[k] >= r.t && jS[k] - r.t <= 600) before = true;
    if (before && r.b >= 0.02f) dipD[nd++] = r.b;
  }

  Serial.printf("Parado: velocidade max %.2f m/s (bom: abaixo de 0.10)\n", stMaxVel);
  Serial.printf("Agachadas (%d):", ns);
  for (int i = 0; i < ns; i++) Serial.printf("  %.0f cm a %.2f m/s", sqD[i] * 100, sqV[i]);
  Serial.printf("\nLevantadas (%d):", nr);
  for (int i = 0; i < nr; i++) Serial.printf("  %.2f m/s", ruV[i]);
  Serial.printf("\nPulos (%d):", nj);
  for (int i = 0; i < nj; i++) Serial.printf("  %.0f ms no ar, impulso %.2f m/s, forca min %.2f g", jT[i], jV[i], jF[i]);
  Serial.printf("\nMergulhos antes do pulo (%d):", nd);
  for (int i = 0; i < nd; i++) Serial.printf("  %.0f cm", dipD[i] * 100);
  Serial.println();

  float nv[NP];
  for (int i = 0; i < NP; i++) { nv[i] = prm[i].val; undoVals[i] = prm[i].val; }
  hasUndo = true;
  Serial.println("\nAvisos:");
  int warns = 0;
  if (ns >= 2) {
    float dmin = minOf(sqD, ns) * 100;                     // cm
    nv[DESCE_VEL] = fminf(prm[DESCE_VEL].padrao, 0.35f * medianOf(sqV, ns));   // so baixa: quem separa agachar de mergulho e a profundidade
    float dc = 0.35f * dmin;
    if (nd > 0) {
      float dipMax = maxOf(dipD, nd) * 100;
      if (dipMax + 1.5f < 0.75f * dmin) dc = dipMax + 1.5f; // mergulho do pulo nunca aperta a seta
      else { warns++; Serial.printf("  - O mergulho antes do pulo (%.0f cm) e quase tao fundo quanto o agachar (%.0f cm): pode abaixar rapidinho antes de pular.\n", dipMax, dmin); }
    }
    nv[DESCE_CM] = constrain(dc, 3.0f, 0.75f * dmin);
    nv[AGACHA_CM] = constrain(nv[DESCE_CM] + 0.4f * (dmin - nv[DESCE_CM]), nv[DESCE_CM] + 2, 0.85f * dmin);
  } else { warns++; Serial.println("  - Menos de 2 agachadas de 8 cm ou mais: mantive os valores do agachar. Agache mais fundo e repita."); }
  if (nr >= 2) nv[SOBE_VEL] = fminf(prm[SOBE_VEL].padrao, 0.40f * medianOf(ruV, nr));
  else { warns++; Serial.println("  - Menos de 2 levantadas medidas: mantive sobe_vel."); }
  if (nj >= 2) { nv[PULO_VEL] = 0.5f * minOf(jV, nj); nv[AR_G] = constrain(maxOf(jF, nj) + 0.25f, 0.30f, 0.50f); }
  else { warns++; Serial.println("  - Menos de 2 pulos medidos: mantive os valores do pulo."); }
  if (stMaxVel > 0.15f) { warns++; Serial.println("  - Parado, a velocidade oscilou: confira se o sensor esta bem preso no case."); }
  if (!accCalOk) { warns++; Serial.println("  - Sensor sem calibracao de fabrica: rode 'sensor' (uma vez) para mais precisao."); }
  if (!warns) Serial.println("  (nenhum)");

  Serial.println("\nValores:            antes   ->  agora");
  for (int i = 0; i < NP; i++) {
    setParam(i, nv[i]);
    if (fabsf(prm[i].val - undoVals[i]) > 0.005f)
      Serial.printf("  %-13s %8.2f  ->  %6.2f\n", prm[i].nome, undoVals[i], prm[i].val);
  }
  saveParams();
  Serial.printf("\nResumo: a seta aperta depois de descer %.0f cm (mergulhos menores sao ignorados)\n", P(DESCE_CM));
  Serial.printf("e so fica apertada se a descida passar de %.0f cm. Pulo: impulso de %.2f m/s + %.0f ms no ar.\n", P(AGACHA_CM), P(PULO_VEL), P(AR_MS));
  Serial.println("Aplicado e salvo. Para voltar ao que era antes: desfazer");
  Serial.println("===================================================");
}

void calTick(unsigned long now) {
  if (calPhase == CAL_OFF) return;
  unsigned long t = now - calT0;
  if (t < 3000) {
    int c = 3 - (int)(t / 1000);
    if (c != calCount) { calCount = c; Serial.printf("  %d...\n", c); }
    return;
  }
  if (!calRec) { calRec = true; Serial.printf("  >>> JA! (%lu s) <<<\n", CAL_DUR[calPhase] / 1000); }
  if (t >= 3000 + CAL_DUR[calPhase]) {
    calRec = false;
    if (calPhase == CAL_STILL) calFinishStill();
    if (calPhase < CAL_JUMP) { calPhase++; calStartPhase(now); }
    else { calPhase = CAL_OFF; calReport(); }
  }
}

// ---------------- Uma amostra (chamado a 200 Hz, com leituras cruas) ----------------
int plotDiv = 0;
void processSample(float rax, float ray, float raz, float gx, float gy, float gz, float dt, unsigned long now) {
  if (scActive) { sensorCalSample(rax, ray, raz, gx, gy, gz, now); return; }
  float ax = (rax - accOff[0]) / accScl[0], ay = (ray - accOff[1]) / accScl[1], az = (raz - accOff[2]) / accScl[2];
  if (fastUntil == 0xFFFFFFFF) {   // logo depois da calibracao do sensor: espera ficar parado
    bool calm = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1) < 0.03f &&
                fabsf(gx - gBias[0]) < 5 && fabsf(gy - gBias[1]) < 5 && fabsf(gz - gBias[2]) < 5;
    if (calm) { if (++stillCnt > 100) { sensorSetFromStill(ax, ay, az, gBias[0], gBias[1], gBias[2], now); stillCnt = 0;
                                          Serial.println("Sensor pronto para jogar."); } }
    else stillCnt = 0;
    return;
  }
  imuStep(ax, ay, az, gx, gy, gz, dt, now);
  jumpStep(now);
  crouchStep(now);
  if (calPhase == CAL_STILL && calRec) calStillSample(ax, ay, az, gx, gy, gz);
  calTick(now);
  if (logMode == 2 && ++plotDiv >= 4) {   // 50 Hz no Serial Plotter
    plotDiv = 0;
    Serial.printf("forca_vert:%.2f,ar_g:%.2f,vel:%.2f,desce_vel:%.2f,sobe_vel:%.2f,descida_m:%.2f,baixo:%d,pulo:%d\n",
                  fv, P(AR_G), vel, -P(DESCE_VEL), P(SOBE_VEL), pTop - pos,
                  crState == CR_DOWN ? -1 : 0, spaceByJump ? 1 : 0);
  }
}

// ======================= [/ALGORITMO] =======================

// ---------------- Dados salvos na placa ----------------
Preferences prefs;
void loadParams() {
  defaultParams();
  prefs.begin("pulo3", true);
  for (int i = 0; i < NP; i++) setParam(i, prefs.getFloat(prm[i].nome, prm[i].padrao));
  accCalOk = prefs.getBool("acc_ok", false);
  if (accCalOk) {
    accOff[0] = prefs.getFloat("acc_ox", 0); accOff[1] = prefs.getFloat("acc_oy", 0); accOff[2] = prefs.getFloat("acc_oz", 0);
    accScl[0] = prefs.getFloat("acc_sx", 1); accScl[1] = prefs.getFloat("acc_sy", 1); accScl[2] = prefs.getFloat("acc_sz", 1);
  }
  prefs.end();
}
void saveParams() {
  prefs.begin("pulo3", false);
  for (int i = 0; i < NP; i++) prefs.putFloat(prm[i].nome, prm[i].val);
  prefs.end();
}
void saveSensorCal() {
  prefs.begin("pulo3", false);
  prefs.putFloat("acc_ox", accOff[0]); prefs.putFloat("acc_oy", accOff[1]); prefs.putFloat("acc_oz", accOff[2]);
  prefs.putFloat("acc_sx", accScl[0]); prefs.putFloat("acc_sy", accScl[1]); prefs.putFloat("acc_sz", accScl[2]);
  prefs.putBool("acc_ok", true);
  prefs.end();
}

// ---------------- Teclado BLE ----------------
const uint8_t hidReportDescriptor[] = {
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,       // Generic Desktop, Keyboard
  0x85, 0x01,                               //   Report ID 1
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,       //   modificadores
  0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01,       //   reservado
  0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, // LEDs
  0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
  0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
  0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, //   ate 6 teclas
  0xC0
};
struct KeyReport { uint8_t modifiers; uint8_t reserved; uint8_t keys[6]; } __attribute__((packed));
const uint8_t KEY_SPACE = 0x2C, KEY_DOWN = 0x51;   // HID Usage Tables, pagina Keyboard

BLEHIDDevice* hid;
BLECharacteristic* inputKeys;
#if defined(CONFIG_BLUEDROID_ENABLED)
BLE2902* cccd = nullptr;                 // a TV/Mac liga as notificacoes aqui quando ativa o teclado
#endif
volatile bool connected = false;
bool spaceDown = false, arrowDown = false;
unsigned long spaceAt = 0, downAt = 0;
uint8_t spaceSrc = 0, downSrc = 0;       // quem esta segurando cada tecla (movimento, botoes...)
volatile int notifyStatus = -1;          // resultado do ultimo envio, informado pela biblioteca BLE
volatile uint32_t notifyCode = 0;
unsigned long keyTestT0 = 0;             // teste de teclas em andamento (0 = nao)

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) { connected = true; }
  void onDisconnect(BLEServer*) {
    connected = false; spaceDown = false; arrowDown = false; spaceSrc = 0; downSrc = 0;
    BLEDevice::startAdvertising();
  }
};

class SecurityCallbacks : public BLESecurityCallbacks {
  bool onSecurityRequest() { return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t a) { Serial.println(a.success ? "Bluetooth: pareado" : "Bluetooth: FALHA no pareamento"); }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* d) { Serial.println(d->sec_state.encrypted ? "Bluetooth: pareado" : "Bluetooth: FALHA no pareamento"); }
#endif
  uint32_t onPassKeyRequest() { return 0; }
  void onPassKeyNotify(uint32_t) {}
  bool onConfirmPIN(uint32_t) { return true; }
};

class KeyNotifyCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic*, Status s, uint32_t code) override { notifyStatus = (int)s; notifyCode = code; }
};

// 0 desconectado | 1 conectado, mas a TV/Mac ainda nao ativou o teclado | 2 teclado ativo
int kbState() {
  if (!connected) return 0;
#if defined(CONFIG_BLUEDROID_ENABLED)
  if (cccd != nullptr && !cccd->getNotifications()) return 1;
#endif
  return 2;
}
const char* kbText(int st) {
  return st == 0 ? "DESCONECTADO - conecte \"Controle Pulo\" no Bluetooth da TV ou do Mac"
       : st == 1 ? "conectado, esperando a TV/Mac ativar o teclado..."
                 : "TECLADO ATIVO - as teclas vao para a janela em foco na TV/Mac";
}

// Envia o estado das teclas e devolve o que aconteceu (para o log)
const char* sendKeys() {
  static char buf[64];
  bool calibrating = calPhase != CAL_OFF || scActive;
  KeyReport r;
  memset(&r, 0, sizeof(r));
  int i = 0;
  if (!calibrating) {                    // calibrando: manda tudo solto
    if (spaceDown) r.keys[i++] = KEY_SPACE;
    if (arrowDown) r.keys[i++] = KEY_DOWN;
  }
  if (!connected) return "NAO enviada: Bluetooth desconectado";
  notifyStatus = -1;
  inputKeys->setValue((uint8_t*)&r, sizeof(r));
  inputKeys->notify();
  if (calibrating) return "NAO enviada: calibrando, teclas desligadas";
  switch (notifyStatus) {
    case BLECharacteristicCallbacks::SUCCESS_NOTIFY:        return "enviada";
    case BLECharacteristicCallbacks::ERROR_NOTIFY_DISABLED:
    case BLECharacteristicCallbacks::ERROR_NO_SUBSCRIBER:   return "NAO enviada: a TV/Mac ainda nao ativou o teclado";
    case BLECharacteristicCallbacks::ERROR_NO_CLIENT:       return "NAO enviada: Bluetooth desconectado";
    case -1:                                                return "enviada (sem confirmacao)";
    default:
      snprintf(buf, sizeof(buf), "NAO enviada: erro do Bluetooth (%d/%lu)", notifyStatus, (unsigned long)notifyCode);
      return buf;
  }
}

// Cada tecla pode ser apertada por varias fontes ao mesmo tempo (movimento, botoes, BOOT, teste).
// Ela fica apertada enquanto QUALQUER fonte estiver apertando.
enum { SRC_MOV = 1, SRC_BOOT = 2, SRC_BTN = 4, SRC_TEST = 8 };

void setKey(bool isSpace, uint8_t src, bool on, const char* origem) {
  uint8_t& m = isSpace ? spaceSrc : downSrc;
  bool was = m != 0;
  m = on ? (m | src) : (m & ~src);
  bool is = m != 0;
  if (was == is) return;                 // outra fonte ja segurava a tecla
  bool& st = isSpace ? spaceDown : arrowDown;
  unsigned long& at = isSpace ? spaceAt : downAt;
  st = is;
  unsigned long now = millis();
  const char* res = sendKeys();
  const char* nome = isSpace ? "ESPACO" : "SETA BAIXO";
  if (is) { at = now; LOG("  >> %s %s%s   [%s]\n", nome, isSpace ? "apertado" : "apertada", origem, res); }
  else LOG("  >> %s %s, ficou %lu ms   [%s]\n", nome, isSpace ? "solto" : "solta", now - at, res);
}
void releaseAllKeys() {
  if (spaceSrc) { spaceSrc = SRC_TEST; setKey(true, SRC_TEST, false, ""); }
  if (downSrc) { downSrc = SRC_TEST; setKey(false, SRC_TEST, false, ""); }
}
// Chamadas pelo algoritmo (ignoradas durante o teste de teclas)
void keySpace(bool on) { if (!keyTestT0) setKey(true, SRC_MOV, on, ""); }
void keyDown(bool on) { if (!keyTestT0) setKey(false, SRC_MOV, on, ""); }

// ---------------- Teste de teclas (comando "teclas") ----------------
struct TestStep { unsigned long at; bool space; bool on; };
const TestStep KEYTEST[] = {
  {5000, true, true},  {5150, true, false},     // ESPACO
  {6000, true, true},  {6150, true, false},     // ESPACO
  {7000, false, true}, {9000, false, false},    // SETA BAIXO segurada 2 s
};
const int KEYTEST_N = sizeof(KEYTEST) / sizeof(KEYTEST[0]);
int keyTestStep = 0, keyTestCount = -1;

void keyTestStart() {
  if (calPhase != CAL_OFF || scActive) { Serial.println("Espere a calibracao terminar."); return; }
  releaseAllKeys();
  logMode = 1;
  keyTestT0 = millis(); if (!keyTestT0) keyTestT0 = 1;
  keyTestStep = 0; keyTestCount = -1;
  Serial.printf("\n######## TESTE DE TECLAS ########\nBluetooth: %s\n", kbText(kbState()));
  Serial.println("Clique AGORA na janela do jogo (ou no testador do guia). Em 5 s:");
  Serial.println("ESPACO 2 vezes e depois SETA BAIXO segurada por 2 s.");
}

void keyTestTick() {
  if (!keyTestT0) return;
  unsigned long t = millis() - keyTestT0;
  if (t < 5000) {
    int c = 5 - (int)(t / 1000);
    if (c != keyTestCount) { keyTestCount = c; Serial.printf("  %d...\n", c); }
    return;
  }
  while (keyTestStep < KEYTEST_N && t >= KEYTEST[keyTestStep].at) {
    const TestStep& st = KEYTEST[keyTestStep++];
    setKey(st.space, SRC_TEST, st.on, " (teste)");
  }
  if (keyTestStep >= KEYTEST_N) {
    keyTestT0 = 0;
    Serial.println("Teste concluido. O personagem pulou 2 vezes e abaixou por 2 s?");
    Serial.println("  - Se as linhas dizem [enviada] e nada aconteceu: o foco nao estava na janela do jogo.");
    Serial.println("  - Se dizem [NAO enviada: ...]: o motivo esta escrito na propria linha.");
  }
}

void bleInit() {
  BLEDevice::init(BT_NAME);
  BLESecurity* sec = new BLESecurity();
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setAuthenticationMode(true, false, true);
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  hid = new BLEHIDDevice(server);
  hid->manufacturer()->setValue("DIY");
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));
  inputKeys = hid->inputReport(1);
  inputKeys->setCallbacks(new KeyNotifyCallbacks());
#if defined(CONFIG_BLUEDROID_ENABLED)
  cccd = (BLE2902*)inputKeys->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
#endif
  hid->outputReport(1);
  hid->setBatteryLevel(100);
  hid->startServices();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->setAppearance(HID_KEYBOARD);
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.printf("Bluetooth pronto: procure \"%s\"\n", BT_NAME);
}

// ---------------- MPU6050 (registradores do datasheet) ----------------
const uint8_t MPU_ADDR = 0x68;
bool mpuOk = false;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU_ADDR, (size_t)1) != 1) return 0xFF;
  return Wire.read();
}
bool mpuRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                  // ACCEL_XOUT_H ... GYRO_ZOUT_L
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (size_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) { uint8_t hi = Wire.read(); uint8_t lo = Wire.read(); r[i] = (int16_t)((hi << 8) | lo); }
  ax = r[0] / 4096.0f; ay = r[1] / 4096.0f; az = r[2] / 4096.0f;   // ±8 g
  gx = r[4] / 32.8f;   gy = r[5] / 32.8f;   gz = r[6] / 32.8f;     // ±1000 graus/s
  return true;
}
void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  mpuWrite(0x6B, 0x80); delay(100);                  // reset
  mpuWrite(0x6B, 0x01); delay(10);                   // acorda, relogio do giroscopio X
  uint8_t who = mpuRead8(0x75);
  mpuOk = who != 0xFF;
  mpuWrite(0x19, 4);                                 // 1 kHz / (1+4) = 200 Hz
  mpuWrite(0x1A, 0x03);                              // filtro 44 Hz (atraso ~5 ms)
  mpuWrite(0x1B, 0x10);                              // giroscopio ±1000 graus/s
  mpuWrite(0x1C, 0x10);                              // acelerometro ±8 g
  if (who == 0x70 || who == 0x71 || who == 0x73) mpuWrite(0x1D, 0x03);   // clones da familia MPU6500
  Serial.printf("Sensor WHO_AM_I = 0x%02X %s\n", who,
                who == 0x68 ? "(MPU6050)" : who == 0xFF ? "(SEM RESPOSTA: confira os 4 fios; o BOOT continua funcionando)"
                                                        : "(chip compativel/clone)");
}

// Espera 1 s parado (ate 10 s) e ajusta o sensor
void sensorBootCal() {
  Serial.println("Ajustando o sensor: deixe o controle PARADO (LED piscando)...");
  float ref[6] = {0}, sum[6] = {0}; int n = 0;
  unsigned long start = millis(), last = 0;
  while (n < 200 && millis() - start < 10000) {
    if (micros() - last < 5000) continue;
    last = micros();
    float s[6];
    if (!mpuRead(s[0], s[1], s[2], s[3], s[4], s[5])) continue;
    for (int k = 0; k < 3; k++) s[k] = (s[k] - accOff[k]) / accScl[k];
    digitalWrite(PIN_LED, (millis() / 100) % 2);
    if (n == 0) for (int i = 0; i < 6; i++) ref[i] = s[i];
    bool moved = false;
    for (int i = 0; i < 3; i++) if (fabsf(s[i] - ref[i]) > 0.05f) moved = true;
    for (int i = 3; i < 6; i++) if (fabsf(s[i] - ref[i]) > 4.0f) moved = true;
    if (moved) { n = 0; for (int i = 0; i < 6; i++) sum[i] = 0; continue; }
    for (int i = 0; i < 6; i++) sum[i] += s[i];
    n++;
  }
  if (n >= 200) {
    for (int i = 0; i < 6; i++) sum[i] /= n;
    sensorSetFromStill(sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], millis());
    Serial.printf("Pronto! Sensor parado mede %.3f g\n", g0);
  } else {
    sensorSetFromStill(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5], millis());
    Serial.println("Nao ficou parado 1 s: comecei assim mesmo, ele se ajusta quando ficar parado.");
  }
  if (!accCalOk) Serial.println(">> Recomendado: digite 'sensor' (uma vez so) para corrigir o erro de fabrica do sensor.");
  digitalWrite(PIN_LED, LOW);
}

// ---------------- Comandos ----------------
void printHelp() {
  Serial.println("\nComandos (Serial Monitor 115200, 'New Line'):");
  Serial.println("  teclas              testa o envio: ESPACO 2x e SETA BAIXO 2 s (clique antes na janela do jogo)");
  Serial.println("  sensor              calibra o erro de fabrica do sensor (UMA vez; 6 posicoes paradas)");
  Serial.println("  cal                 calibra os movimentos da crianca (parado, agachar, pular) e aplica");
  Serial.println("  desfazer            volta os valores de antes da ultima 'cal'");
  Serial.println("  ver                 mostra parametros e estado do sensor");
  Serial.println("  <nome> <valor>      muda um parametro. Ex.: desce_vel 0.15");
  Serial.println("  log 0 | 1 | 2       silencio | eventos (padrao) | grafico (Serial Plotter)");
  Serial.println("  padrao              volta os parametros de movimento aos valores de fabrica");
  Serial.println("Lembrete: as teclas vao para a janela EM FOCO. Depois de digitar um comando aqui,");
  Serial.println("clique na janela do jogo antes de se mexer (o Serial Monitor continua mostrando o log).");
}

void printStatus() {
  printParams();
  Serial.printf("\nSensor: %s | offset (%+.3f, %+.3f, %+.3f) g | escala (%.3f, %.3f, %.3f)\n",
                accCalOk ? "calibrado" : "SEM calibracao de fabrica (rode 'sensor')",
                accOff[0], accOff[1], accOff[2], accScl[0], accScl[1], accScl[2]);
  Serial.printf("Parado mede %.3f g | giroscopio parado (%.1f, %.1f, %.1f) graus/s\n", g0, gBias[0], gBias[1], gBias[2]);
  Serial.printf("Agora: forca vertical %.2f g | velocidade %.2f m/s | %s | %s\n",
                fv, vel, still ? "parado" : "mexendo", crState == CR_DOWN ? "ABAIXADO" : "em pe");
  Serial.printf("Bluetooth: %s\n", kbText(kbState()));
}

void handleCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase(); cmd.replace(',', '.');
  if (!cmd.length()) return;
  int sp = cmd.indexOf(' ');
  String w = sp < 0 ? cmd : cmd.substring(0, sp);
  String arg = sp < 0 ? "" : cmd.substring(sp + 1);
  arg.trim();
  if (w == "ajuda" || w == "?" || w == "help") printHelp();
  else if (w == "ver" || w == "params") printStatus();
  else if (w == "teclas" || w == "teste") keyTestStart();
  else if (w == "sensor") { calPhase = CAL_OFF; calRec = false; sensorCalStart(millis()); }
  else if (w == "cal") {
    if (scActive) Serial.println("Termine primeiro a calibracao do sensor (ou reinicie a placa).");
    else calStart(millis());
  }
  else if (w == "desfazer") {
    if (!hasUndo) { Serial.println("Nada para desfazer."); return; }
    for (int i = 0; i < NP; i++) prm[i].val = undoVals[i];
    saveParams(); hasUndo = false; Serial.println("Valores anteriores restaurados."); printParams();
  }
  else if (w == "padrao") { defaultParams(); saveParams(); Serial.println("Valores de fabrica restaurados."); printParams(); }
  else if (w == "log") { logMode = constrain(arg.toInt(), 0, 2); Serial.printf("log = %d\n", logMode); }
  else {
    for (int i = 0; i < NP; i++) {
      if (w == prm[i].nome) {
        if (!arg.length()) { Serial.printf("%s = %.2f  (%s)\n", prm[i].nome, prm[i].val, prm[i].ajuda); return; }
        setParam(i, arg.toFloat()); saveParams();
        Serial.printf("%s = %.2f (salvo)\n", prm[i].nome, prm[i].val);
        return;
      }
    }
    Serial.println("Nao entendi. Digite: ajuda");
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BTN_PULO, INPUT_PULLUP);    // sem botao ligado, o pino fica em 1: nada acontece
  pinMode(PIN_BTN_BAIXO, INPUT_PULLUP);
  delay(300);
  Serial.println("\n=== Controle do Pulo v3.2 ===");
  loadParams();
  mpuInit();
  bleInit();
  if (mpuOk) sensorBootCal();
  printHelp();
}

void loop() {
  static uint32_t lastUs = 0;
  static int lastKb = -1;
  static String line;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (line.length()) handleCommand(line); line = ""; }
    else if (line.length() < 64) line += c;
  }

  int kb = kbState();                                    // avisa quando o Bluetooth muda
  if (kb != lastKb) { lastKb = kb; Serial.printf("Bluetooth: %s\n", kbText(kb)); }

  // Botoes (com anti-repique de 30 ms): BOOT e botao 1 = ESPACO, botao 2 = SETA BAIXO
  struct Btn { int pin; bool isSpace; uint8_t src; const char* nome; bool was; unsigned long t; };
  static Btn btns[] = {
    {PIN_BOOT, true, SRC_BOOT, " (botao BOOT)", false, 0},
    {PIN_BTN_PULO, true, SRC_BTN, " (botao 1)", false, 0},
    {PIN_BTN_BAIXO, false, SRC_BTN, " (botao 2)", false, 0},
  };
  for (Btn& b : btns) {
    bool on = digitalRead(b.pin) == LOW;
    if (on != b.was && millis() - b.t > 30) {
      b.was = on; b.t = millis();
      if (!keyTestT0) setKey(b.isSpace, b.src, on, b.nome);
    }
  }
  keyTestTick();

  uint32_t nowUs = micros();
  if (nowUs - lastUs < 5000) return;                     // 200 amostras por segundo
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.02f) dt = 0.02f;

  // LED: pisca rapido = calibrando o sensor | piscada curta a cada 1 s = teclado NAO ativo
  //      aceso = tecla apertada | apagado = pronto
  bool led;
  if (scActive) led = (millis() / 250) % 2;
  else led = spaceDown || arrowDown || (kb != 2 && millis() % 1000 < 100);
  digitalWrite(PIN_LED, led ? HIGH : LOW);
  if (!mpuOk) return;

  float ax, ay, az, gx, gy, gz;
  static int fails = 0;
  if (!mpuRead(ax, ay, az, gx, gy, gz)) {
    if (++fails == 200) Serial.println("Aviso: o sensor parou de responder (confira os fios).");
    return;
  }
  fails = 0;
  processSample(ax, ay, az, gx, gy, gz, dt, millis());
}
