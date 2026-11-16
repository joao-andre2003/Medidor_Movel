/*
  Medidor de energia - ESP32-C6 - leitura por ADC continuo (DMA)

  Ligacoes (circuito do TCC):
    GPIO2 -> V        (modulo ZMPT101B, com filtro 1k/100nF)
    GPIO0 -> I_ALTA   (SCT013-030 -> LM324 ganho -0,68, ate ~30 A)
    GPIO1 -> I_BAIXA  (SCT013-030 -> LM324 ganho -6,8,  ate ~3 A)

  A cada janela (~267 ms) o codigo:
    1. Amostra os 3 canais com taxa fixa via DMA (128 amostras/ciclo por canal)
    2. Converte raw -> volts com a calibracao de fabrica do ADC (eFuse)
    3. Remove o DC e acha ciclos inteiros pelos cruzamentos por zero de V
    4. Escolhe a faixa de corrente (baixa se nao saturou, senao alta)
    5. Calcula Vrms, Irms, P, S, FP, Q1, defasagem, f, THD e energia
    6. Corrige o atraso da amostragem sequencial e a calibracao de fase

  Comandos no Serial Monitor (modo "Newline", 115200):
    ajuda                -> lista os comandos
    dump V               -> imprime a ultima janela do canal (volts no pino)
    dump I_ALTA          -> idem, faixa alta
    dump I_BAIXA         -> idem, faixa baixa
    fs                   -> mede a taxa de amostragem real (10 s)
    cal                  -> liga/desliga a linha extra com valores de calibracao
    zera                 -> zera o contador de energia

  Requer Arduino-ESP32 core 3.x (ESP-IDF 5.1 ou mais novo).
  Nao use analogRead() junto com este codigo.
*/

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

// =========================== CONFIGURACAO ===========================
constexpr uint32_t FS_TOTAL_HZ    = 23040;   // taxa total do ADC = 3 canais x 7680 Hz
constexpr float    FS_CORRECAO    = 1.0f;    // fs_real / fs_configurada (comando "fs")
constexpr int      N_CH           = 3;
constexpr float    FS_CANAL       = FS_TOTAL_HZ * FS_CORRECAO / N_CH;
constexpr int      N_WIN          = 2048;    // amostras por canal por janela (~267 ms, ~16 ciclos)
constexpr int      N_DESCARTE     = 16;      // amostras descartadas no inicio de cada janela
constexpr int      MARGEM         = 10;      // margem nas bordas (para interpolacao de fase)
constexpr int      N_HARM         = 15;      // harmonicas usadas no THD
constexpr float    F_NOM          = 60.0f;
constexpr float    V_MIN_PINO     = 0.010f;  // Vrms no pino abaixo disso = "sem tensao"
constexpr float    LIMITE_BAIXA_V = 0.90f;   // pico da faixa baixa (em torno do DC) que forca a faixa alta
constexpr float    I_MIN_THD      = 0.05f;   // abaixo disso (A) nao calcula THD de corrente

constexpr uint32_t FRAME_BYTES    = 256 * SOC_ADC_DIGI_RESULT_BYTES;

struct Canal {
  const char* nome;
  int   gpio;
  float escala;      // grandeza fisica por volt no pino (negativo = canal invertido)
  float ganho_cal;   // ajuste fino de ganho (1.0 = sem ajuste)
  float fase_cal;    // graus medidos com carga resistiva (so corrente)
  float ruido_rms;   // ruido RMS sem carga, em A (so corrente)
  adc_channel_t ch;  // preenchido no setup
};

enum { CV = 0, CIA = 1, CIB = 2 };  // a ordem aqui = ordem de amostragem

Canal canais[N_CH] = {
  // nome        gpio  escala           ganho_cal  fase_cal  ruido_rms
  { "V",         2,    1.0f,            1.0f,      0.0f,     0.0f },  // CALIBRAR: escala = V_multimetro / Vpino_rms
  { "I_ALTA",    0,   -30.0f / 0.68f,   1.0f,      0.0f,     0.0f },  // SCT013-030 (1 V = 30 A), ganho 0,68
  { "I_BAIXA",   1,   -30.0f / 6.8f,    1.0f,      0.0f,     0.0f },  // SCT013-030 (1 V = 30 A), ganho 6,8
};
// Se usou Rin 15k / Rf 10k e 100k, troque 0.68f por 0.667f e 6.8f por 6.67f.
// ====================================================================

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
  #define ADC_FORMATO   ADC_DIGI_OUTPUT_FORMAT_TYPE1
  #define ADC_CANAL(p)  ((p)->type1.channel)
  #define ADC_DADO(p)   ((p)->type1.data)
#else
  #define ADC_FORMATO   ADC_DIGI_OUTPUT_FORMAT_TYPE2
  #define ADC_CANAL(p)  ((p)->type2.channel)
  #define ADC_DADO(p)   ((p)->type2.data)
#endif

static adc_continuous_handle_t g_adc = nullptr;
static uint8_t  g_frame[FRAME_BYTES];
static float    g_buf[N_CH][N_WIN];   // volts no pino (sem DC depois do processamento)
static float    g_dc[N_CH];           // DC removido de cada canal (V)
static uint16_t g_lut[N_CH][4096];    // raw -> mV (calibracao de fabrica)
static int8_t   g_idx_do_canal[16];   // canal do ADC -> indice em canais[]
static float    g_cos[N_WIN], g_sin[N_WIN];
static bool     g_modo_cal = false;
static double   g_energia_Wh = 0;
static int64_t  g_t_ultimo_us = 0;

struct Resultado {
  bool  tem_tensao;
  int   faixa;                 // CIA ou CIB
  int   ciclos, n_amostras;
  float f, vrms, irms, p, s, fp, p1, q1, phi, phi_bruto, thd_v, thd_i;
  float vpino_rms[N_CH], pico_baixa, ia_bruto, ib_bruto;
};

// ----------------------------- utilitarios -----------------------------
static void falha(const char* msg, esp_err_t e) {
  Serial.printf("[ERRO] %s: %s\n", msg, esp_err_to_name(e));
  while (true) delay(1000);
}

static inline float interp(const float* x, float pos) {
  int k = (int)floorf(pos);
  float a = pos - k;
  return x[k] + a * (x[k + 1] - x[k]);
}

static float normaliza(float g) {
  while (g > 180.0f)   g -= 360.0f;
  while (g <= -180.0f) g += 360.0f;
  return g;
}

// ------------------------------ ADC ------------------------------
static void iniciaADC() {
  memset(g_idx_do_canal, -1, sizeof(g_idx_do_canal));

  adc_continuous_handle_cfg_t hcfg = {};
  hcfg.max_store_buf_size = 8192;
  hcfg.conv_frame_size    = FRAME_BYTES;
  esp_err_t e = adc_continuous_new_handle(&hcfg, &g_adc);
  if (e != ESP_OK) falha("adc_continuous_new_handle", e);

  adc_digi_pattern_config_t pat[N_CH] = {};
  for (int i = 0; i < N_CH; ++i) {
    adc_unit_t unit;
    e = adc_continuous_io_to_channel(canais[i].gpio, &unit, &canais[i].ch);
    if (e != ESP_OK || unit != ADC_UNIT_1) falha("GPIO sem ADC1", e);
    pat[i].atten     = ADC_ATTEN_DB_12;
    pat[i].channel   = canais[i].ch & 0x7;
    pat[i].unit      = ADC_UNIT_1;
    pat[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    g_idx_do_canal[canais[i].ch] = i;
  }

  adc_continuous_config_t cfg = {};
  cfg.pattern_num    = N_CH;
  cfg.adc_pattern    = pat;
  cfg.sample_freq_hz = FS_TOTAL_HZ;
  cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  cfg.format         = ADC_FORMATO;
  e = adc_continuous_config(g_adc, &cfg);
  if (e != ESP_OK) falha("adc_continuous_config", e);
}

// Tabela raw -> mV usando a calibracao gravada de fabrica no chip
static void criaLUT() {
  for (int i = 0; i < N_CH; ++i) {
    adc_cali_handle_t h = nullptr;
    esp_err_t e = ESP_FAIL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t c = {};
    c.unit_id  = ADC_UNIT_1;
    c.chan     = canais[i].ch;
    c.atten    = ADC_ATTEN_DB_12;
    c.bitwidth = ADC_BITWIDTH_12;
    e = adc_cali_create_scheme_curve_fitting(&c, &h);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t c = {};
    c.unit_id  = ADC_UNIT_1;
    c.atten    = ADC_ATTEN_DB_12;
    c.bitwidth = ADC_BITWIDTH_12;
    e = adc_cali_create_scheme_line_fitting(&c, &h);
#endif
    if (e != ESP_OK)
      Serial.printf("[AVISO] %s sem calibracao de fabrica, usando conversao linear.\n", canais[i].nome);

    for (int r = 0; r < 4096; ++r) {
      int mv = 0;
      if (e == ESP_OK && adc_cali_raw_to_voltage(h, r, &mv) == ESP_OK) g_lut[i][r] = (uint16_t)mv;
      else g_lut[i][r] = (uint16_t)(r * 3300L / 4095);
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (e == ESP_OK) adc_cali_delete_scheme_curve_fitting(h);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (e == ESP_OK) adc_cali_delete_scheme_line_fitting(h);
#endif
  }
}

// Enche g_buf com N_WIN amostras de cada canal (mesmo instante de inicio)
static bool capturaJanela() {
  int cnt[N_CH] = {0}, desc[N_CH] = {0};
  bool ok = false;

  adc_continuous_start(g_adc);
  const int64_t t0 = esp_timer_get_time();

  while (esp_timer_get_time() - t0 < 2000000) {
    uint32_t n = 0;
    esp_err_t e = adc_continuous_read(g_adc, g_frame, FRAME_BYTES, &n, 200);
    if (e != ESP_OK) continue;

    for (uint32_t k = 0; k < n; k += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t* p = (adc_digi_output_data_t*)&g_frame[k];
      uint32_t ch = ADC_CANAL(p);
      if (ch >= 16) continue;
      int idx = g_idx_do_canal[ch];
      if (idx < 0) continue;
      if (desc[idx] < N_DESCARTE) { ++desc[idx]; continue; }
      if (cnt[idx] < N_WIN) g_buf[idx][cnt[idx]++] = g_lut[idx][ADC_DADO(p) & 0xFFF] * 0.001f;
    }

    ok = true;
    for (int c = 0; c < N_CH; ++c) if (cnt[c] < N_WIN) ok = false;
    if (ok) break;
  }

  adc_continuous_stop(g_adc);
  return ok;
}

// ---------------------------- processamento ----------------------------
// DFT nas harmonicas 1..N_HARM (janela com numero inteiro de ciclos)
static void dft(const float* x, int n0, int nw, int ciclos, float* rms_h, float* fase1_graus) {
  for (int h = 1; h <= N_HARM; ++h) {
    const int passo = (ciclos * h) % nw;
    int idx = 0;
    float re = 0, im = 0;
    for (int m = 0; m < nw; ++m) {
      re += x[n0 + m] * g_cos[idx];
      im -= x[n0 + m] * g_sin[idx];
      idx += passo;
      if (idx >= nw) idx -= nw;
    }
    rms_h[h] = sqrtf(re * re + im * im) * 1.41421356f / nw;
    if (h == 1) *fase1_graus = atan2f(im, re) * RAD_TO_DEG;
  }
}

static Resultado processa() {
  Resultado r = {};

  // 1) Remove o DC da janela toda
  for (int c = 0; c < N_CH; ++c) {
    float s = 0;
    for (int n = 0; n < N_WIN; ++n) s += g_buf[c][n];
    g_dc[c] = s / N_WIN;
    for (int n = 0; n < N_WIN; ++n) g_buf[c][n] -= g_dc[c];
  }
  const float* v = g_buf[CV];

  // 2) Escolhe a faixa de corrente pelo pico da faixa baixa
  float pico = 0;
  for (int n = 0; n < N_WIN; ++n) pico = fmaxf(pico, fabsf(g_buf[CIB][n]));
  r.pico_baixa = pico;
  r.faixa = (pico > LIMITE_BAIXA_V) ? CIA : CIB;

  // 3) Ha tensao? Cruzamentos por zero (subida) em V -> ciclos inteiros
  float sv = 0;
  for (int n = 0; n < N_WIN; ++n) sv += v[n] * v[n];
  bool tem = sqrtf(sv / N_WIN) > V_MIN_PINO;

  int n0 = MARGEM, n1 = N_WIN - MARGEM, nc = 0;
  float x0 = 0, x1 = 0;
  if (tem) {
    const int dist_min = (int)(0.6f * FS_CANAL / F_NOM);
    int ultimo = -dist_min, primeiro_n = -1, ultimo_n = -1;
    for (int n = MARGEM; n < N_WIN - MARGEM; ++n) {
      if (v[n - 1] < 0 && v[n] >= 0 && n - ultimo >= dist_min) {
        float x = (n - 1) + v[n - 1] / (v[n - 1] - v[n]);  // instante fracionario do cruzamento
        if (nc == 0) { primeiro_n = n; x0 = x; }
        ultimo_n = n; x1 = x; ++nc; ultimo = n;
      }
    }
    if (nc >= 2) { n0 = primeiro_n; n1 = ultimo_n; }
    else tem = false;
  }
  r.tem_tensao = tem;
  if (tem) {
    r.ciclos = nc - 1;
    r.f = r.ciclos * FS_CANAL / (x1 - x0);
  } else {
    r.ciclos = max(1, (int)lroundf((n1 - n0) * F_NOM / FS_CANAL));
    r.f = 0;
  }
  const int nw = n1 - n0;
  r.n_amostras = nw;
  const float f_uso = tem ? r.f : F_NOM;

  // 4) Recentra em ciclos inteiros e calcula o RMS no pino
  for (int c = 0; c < N_CH; ++c) {
    float s = 0;
    for (int n = n0; n < n1; ++n) s += g_buf[c][n];
    const float m = s / nw;
    for (int n = 0; n < N_WIN; ++n) g_buf[c][n] -= m;
    g_dc[c] += m;
    float s2 = 0;
    for (int n = n0; n < n1; ++n) s2 += g_buf[c][n] * g_buf[c][n];
    r.vpino_rms[c] = sqrtf(s2 / nw);
  }

  // 5) Grandezas fisicas
  const Canal& cv = canais[CV];
  const Canal& ci = canais[r.faixa];
  const float kv = cv.escala * cv.ganho_cal;
  const float ki = ci.escala * ci.ganho_cal;
  r.vrms = fabsf(kv) * r.vpino_rms[CV];
  const float irms_bruto = fabsf(ki) * r.vpino_rms[r.faixa];
  r.irms = sqrtf(fmaxf(0.0f, irms_bruto * irms_bruto - ci.ruido_rms * ci.ruido_rms));
  r.ia_bruto = fabsf(canais[CIA].escala * canais[CIA].ganho_cal) * r.vpino_rms[CIA];
  r.ib_bruto = fabsf(canais[CIB].escala * canais[CIB].ganho_cal) * r.vpino_rms[CIB];

  // 6) Potencia ativa: V interpolada no instante da amostra de I
  //    atraso da amostragem sequencial (posicao no padrao) - calibracao de fase
  float d = (float)r.faixa / N_CH - ci.fase_cal / 360.0f * FS_CANAL / f_uso;
  d = fminf(fmaxf(d, -(MARGEM - 2)), (float)(MARGEM - 2));
  const float* i = g_buf[r.faixa];
  double sp = 0;
  for (int n = n0; n < n1; ++n) sp += interp(v, n + d) * i[n];
  r.p = kv * ki * (float)(sp / nw);

  // 7) Potencia aparente e fator de potencia
  r.s  = r.vrms * r.irms;
  r.fp = (r.s > 1e-3f) ? fminf(fmaxf(r.p / r.s, -1.0f), 1.0f) : 0.0f;

  // 8) DFT: fundamental (fase, P1, Q1) e harmonicas (THD)
  for (int m = 0; m < nw; ++m) {
    const float a = TWO_PI * m / nw;
    g_cos[m] = cosf(a);
    g_sin[m] = sinf(a);
  }
  float hv[N_HARM + 1], hi[N_HARM + 1], fv = 0, fi = 0;
  dft(v, n0, nw, r.ciclos, hv, &fv);
  dft(i, n0, nw, r.ciclos, hi, &fi);
  if (kv < 0) fv += 180.0f;
  if (ki < 0) fi += 180.0f;

  const float atraso_graus = 360.0f * f_uso * r.faixa / (FS_TOTAL_HZ * FS_CORRECAO);
  r.phi_bruto = normaliza(fv - fi + atraso_graus);
  r.phi       = normaliza(r.phi_bruto - ci.fase_cal);  // + = indutivo (V adiantada)

  const float v1 = fabsf(kv) * hv[1], i1 = fabsf(ki) * hi[1];
  r.p1 = v1 * i1 * cosf(r.phi * DEG_TO_RAD);
  r.q1 = v1 * i1 * sinf(r.phi * DEG_TO_RAD);

  float sh_v = 0, sh_i = 0;
  for (int h = 2; h <= N_HARM; ++h) { sh_v += hv[h] * hv[h]; sh_i += hi[h] * hi[h]; }
  r.thd_v = (hv[1] > 1e-6f) ? 100.0f * sqrtf(sh_v) / hv[1] : 0.0f;
  r.thd_i = (r.irms >= I_MIN_THD && hi[1] > 1e-6f) ? 100.0f * sqrtf(sh_i) / hi[1] : NAN;

  return r;
}

// Energia: P da janela x tempo desde a ultima janela (supoe P constante no intervalo)
static void acumulaEnergia(const Resultado& r) {
  const int64_t agora = esp_timer_get_time();
  if (g_t_ultimo_us != 0) g_energia_Wh += r.p * (double)(agora - g_t_ultimo_us) / 3.6e9;
  g_t_ultimo_us = agora;
}

// ------------------------------ saida ------------------------------
static void imprime(const Resultado& r) {
  if (!r.tem_tensao) Serial.print("[sem tensao] ");
  Serial.printf("V=%.2f V | I=%.3f A (%s) | P=%.1f W | S=%.1f VA | Q1=%.1f var | FP=%.3f | phi=%.2f graus | f=%.3f Hz | THDv=%.2f%% | THDi=",
                r.vrms, r.irms, r.faixa == CIA ? "alta" : "baixa", r.p, r.s, r.q1, r.fp, r.phi, r.f, r.thd_v);
  if (isnan(r.thd_i)) Serial.print("--");
  else Serial.printf("%.2f%%", r.thd_i);
  Serial.printf(" | E=%.5f kWh\n", g_energia_Wh / 1000.0);

  if (g_modo_cal) {
    Serial.printf("  [cal] Vpino_rms: V=%.5f IA=%.5f IB=%.5f V | DC: V=%.4f IA=%.4f IB=%.4f V | pico_IB=%.3f V"
                  " | I_alta=%.4f A I_baixa=%.4f A | phi_bruto=%.2f graus | ciclos=%d N=%d\n",
                  r.vpino_rms[CV], r.vpino_rms[CIA], r.vpino_rms[CIB], g_dc[CV], g_dc[CIA], g_dc[CIB],
                  r.pico_baixa, r.ia_bruto, r.ib_bruto, r.phi_bruto, r.ciclos, r.n_amostras);
  }
}

static void dump(int c) {
  Serial.printf("# canal=%s fs_canal=%.2f Hz N=%d (volts no pino)\n", canais[c].nome, FS_CANAL, N_WIN);
  for (int n = 0; n < N_WIN; ++n) {
    if (n) Serial.print(", ");
    Serial.print(g_buf[c][n] + g_dc[c], 6);
  }
  Serial.println("\n");
}

static void medeFs() {
  Serial.println("Medindo a taxa de amostragem por 10 s...");
  uint32_t n = 0;
  uint64_t total = 0;

  adc_continuous_start(g_adc);
  adc_continuous_read(g_adc, g_frame, FRAME_BYTES, &n, 1000);               // espera o primeiro frame
  while (adc_continuous_read(g_adc, g_frame, FRAME_BYTES, &n, 0) == ESP_OK) {} // esvazia o buffer
  const int64_t t0 = esp_timer_get_time();
  while (esp_timer_get_time() - t0 < 10000000) {
    if (adc_continuous_read(g_adc, g_frame, FRAME_BYTES, &n, 100) == ESP_OK)
      total += n / SOC_ADC_DIGI_RESULT_BYTES;
  }
  const int64_t t1 = esp_timer_get_time();
  adc_continuous_stop(g_adc);

  const double fs_real = total * 1e6 / (double)(t1 - t0);
  Serial.printf("fs configurada = %u Hz | fs medida = %.1f Hz | FS_CORRECAO sugerido = %.5f\n",
                FS_TOTAL_HZ, fs_real, fs_real / FS_TOTAL_HZ);
  Serial.println("Confira tambem: a media de f ao longo de alguns minutos deve dar 60,00 Hz.");
}

static void ajuda() {
  Serial.println("Comandos: ajuda | dump V | dump I_ALTA | dump I_BAIXA | fs | cal | zera");
}

static void trataSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("ajuda")) ajuda();
  else if (cmd.equalsIgnoreCase("fs")) medeFs();
  else if (cmd.equalsIgnoreCase("cal")) {
    g_modo_cal = !g_modo_cal;
    Serial.printf("Modo calibracao: %s\n", g_modo_cal ? "ligado" : "desligado");
  }
  else if (cmd.equalsIgnoreCase("zera")) {
    g_energia_Wh = 0;
    Serial.println("Energia zerada.");
  }
  else if (cmd.startsWith("dump")) {
    String nome = cmd.substring(4);
    nome.trim();
    for (int c = 0; c < N_CH; ++c)
      if (nome.equalsIgnoreCase(canais[c].nome)) { dump(c); return; }
    Serial.println("[ERRO] Canal desconhecido. Use: dump V | dump I_ALTA | dump I_BAIXA");
  }
  else Serial.println("[ERRO] Comando desconhecido. Digite 'ajuda'.");
}

// ------------------------------ Arduino ------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 5000) delay(10);
  delay(300);
  Serial.println("\n=== Medidor de energia - ADC continuo ===");

  iniciaADC();
  criaLUT();

  Serial.printf("fs total = %u Hz | fs por canal = %.2f Hz | %.1f amostras/ciclo | janela = %d amostras (%.0f ms)\n",
                FS_TOTAL_HZ, FS_CANAL, FS_CANAL / F_NOM, N_WIN, N_WIN * 1000.0f / FS_CANAL);
  ajuda();
}

void loop() {
  trataSerial();

  if (!capturaJanela()) {
    Serial.println("[ERRO] Janela incompleta (ADC nao entregou amostras).");
    delay(500);
    return;
  }

  Resultado r = processa();
  acumulaEnergia(r);
  imprime(r);
}
