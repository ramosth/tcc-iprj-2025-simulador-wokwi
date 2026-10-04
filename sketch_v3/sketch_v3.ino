/*
 * =====================================================================================
 *  SISTEMA DE ALERTA PARA BARRAGEM DE REJEITOS — SIMULAÇÃO WOKWI (ESP32) — versão 3
 *  TCC UERJ-IPRJ | Thamires Ramos dos Santos
 *
 *  Modelo de alerta (limiar hidrometeorológico de duas variáveis):
 *    Eq. 1  S   = (L - L_seco) / (L_sat - L_seco)            saturação relativa (0 a 1)
 *    Eq. 2  P72 = P48,obs + P24,prev                          chuva da janela de 72 h
 *    Eq. 3  VERMELHO se S >= S2 e P72 >= P2
 *           AMARELO  se S >= S1 e P72 >= P1 (e não vermelho)
 *           VERDE    nos demais casos
 *  Contingências (cap4, tab:contingencia):
 *    higrômetro com falha  -> só chuva: P72 >= P1 AMARELO; P72 >= P2 VERMELHO
 *    chuva incompleta      -> parcela não obtida OU dia sem registro (null) no BNDMET:
 *                             P72 mínimo = soma do que foi obtido (chuva não é negativa).
 *                             Mínimo >= P1: regra aplicada com o mínimo ("dados de chuva
 *                             incompletos"); mínimo < P1: como sem dados de chuva
 *    sem dados de chuva    -> só solo: S >= S2 AMARELO; senão VERDE ("sem dados de chuva")
 *    sem os dois           -> AMARELO por precaução (decisão do orientador)
 *  Dia null no BNDMET (I006) = dado AUSENTE (não é 0 mm): fica fora da soma e deixa P48
 *  incompleto. P48 só fica indisponível quando a API não responde (HTTP/conexão/JSON).
 *
 *  Hardware (diagram.json):
 *    Potenciômetro pot1 -> GPIO34 (representa o higrômetro)
 *    LED verde GPIO18 | LED amarelo GPIO14 | LED vermelho GPIO27 | buzzer GPIO26
 *    Chave sw_rede -> GPIO32 (HIGH = rede elétrica presente)
 *    Potenciômetro pot_bateria -> GPIO35 (representa a tensão da bateria, 2,50–4,20 V)
 *    LED REDE GPIO25 | LED BACKUP GPIO22 | LED ATENÇÃO GPIO23
 *
 *  Simulação não é medição: os valores de S e de tensão vêm de potenciômetros.
 *
 *  Comandos serial: status | calibrar [seco|sat|reset] | chuva <mm>|parcial <mm>|off|na | help
 *  Chaves e endereços: secrets.h (fora do Git). Modelo: secrets.h.example.
 * =====================================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include "secrets.h" // WIFI_SSID, WIFI_PASS, API_BASE_URL, BNDMET_API_KEY, OWM_API_KEY

// =====================================================================================
//  MODO DEMONSTRAÇÃO
//  Divide TODOS os tempos do sistema (intervalos de leitura e tempo de recarga) por
//  FATOR_DEMO. Com 1, o firmware opera nos tempos reais (60 min / 10 min / 19,5 h).
//  Com 120: 60 min -> 30 s; 10 min -> 5 s; 19,5 h -> 9 min 45 s.
//  É recurso de TESTE da simulação; o texto do TCC deve dizer isso.
// =====================================================================================
const uint32_t FATOR_DEMO = 120; // Fonte: plano v3, Parte 2.6 (modo demonstração; não é parâmetro do modelo)

// =====================================================================================
//  PINOS (diagram.json)
// =====================================================================================
const int PIN_HIGROMETRO = 34;   // Fonte: espressif_esp32 (GPIO34 = entrada ADC1)
const int PIN_BATERIA_ADC = 35;  // Fonte: espressif_esp32 (GPIO35 = entrada ADC1)
const int PIN_REDE = 32;         // Fonte: wokwi_slideswitch (pino comum da chave)
const int PIN_LED_VERDE = 18;    // Fonte: espressif_esp32 (GPIO de saída)
const int PIN_LED_AMARELO = 14;  // Fonte: espressif_esp32 (GPIO de saída)
const int PIN_LED_VERMELHO = 27; // Fonte: espressif_esp32 (GPIO de saída)
const int PIN_BUZZER = 26;       // Fonte: espressif_esp32 (GPIO de saída; sinal do buzzer)
const int PIN_LED_REDE = 25;     // Fonte: espressif_esp32 (GPIO de saída)
const int PIN_LED_BACKUP = 22;   // Fonte: espressif_esp32 (GPIO de saída)
const int PIN_LED_ATENCAO = 23;  // Fonte: espressif_esp32 (GPIO de saída)

// =====================================================================================
//  CONSTANTES DO MODELO (Eq. 1–3)
// =====================================================================================
const int ADC_MAX = 4095;      // Fonte: espressif_esp32 (ADC SAR de 12 bits: 0 a 4095)
const float S1 = 0.64f;        // Fonte: mirus2018developing (Tabela 1, Seattle: limiar inferior de saturação)
const float S2 = 0.86f;        // Fonte: mirus2018developing (Tabela 1, Seattle: limiar superior de saturação)
const float P1_MM = 60.0f;     // Fonte: mendes2020proposicao (72 h, nível de atenção, Campos do Jordão)
const float P2_MM = 100.0f;    // Fonte: mendes2020proposicao (72 h, nível de alerta, Campos do Jordão)
const int DIAS_P48 = 2;        // Fonte: mirus2018developing (janela de t-48 h; 2 totais diários I006)
const int BLOCOS_P24 = 8;      // Fonte: mirus2018developing (janela até t+24 h) + openweathermap_forecast5 (passo de 3 h: 8 x 3 h)

// Calibração inicial do potenciômetro (Eq. 1). Substituída pelo comando "calibrar".
// No Wokwi o potenciômetro é linear; no higrômetro real a escala é invertida
// (seco = leitura alta), o que a Eq. 1 já trata, pois só usa a diferença.
const int L_SECO_PADRAO = 0;   // Fonte: espressif_esp32 (início da escala do ADC; valor de calibração)
const int L_SAT_PADRAO = 4095; // Fonte: espressif_esp32 (fim da escala do ADC; valor de calibração)

// Buzzer: o físico (TMB-12A03) é ATIVO e soa com nível alto. A peça "wokwi-buzzer" é
// piezoelétrica e só soa com sinal oscilante (wokwi_buzzer); por isso a simulação gera
// uma onda quadrada na frequência de ressonância do TMB-12A03.
const unsigned int FREQ_BUZZER_HZ = 2300; // Fonte: quickteck_tmb12a03 (ressonância 2300 ± 500 Hz)

// =====================================================================================
//  INTERVALOS DE LEITURA E ENVIO
// =====================================================================================
const uint32_t INTERVALO_NORMAL_MS = 60UL * 60UL * 1000UL / FATOR_DEMO; // Fonte: freitas2021analise (60 min sem chuva)
const uint32_t INTERVALO_EVENTO_MS = 10UL * 60UL * 1000UL / FATOR_DEMO; // Fonte: freitas2021analise (10 min com chuva)

// =====================================================================================
//  ENERGIA (simulação — Parte 3 do plano; valores do P4)
// =====================================================================================
const int VBAT_MIN_MV = 2500;     // Fonte: panasonic_ncr18650b (tensão de corte de descarga, 2,50 V)
const int VBAT_MAX_MV = 4200;     // Fonte: panasonic_ncr18650b (tensão de fim de carga CC-CV, 4,20 V)
const int VBAT_BAIXA_MV = 3300;   // Fonte: panasonic_ncr18650b (curva de descarga 0,2 C: joelho em ~3,3 V)
const int VBAT_CRITICA_MV = 3000; // Fonte: panasonic_ncr18650b (curva de descarga 0,2 C: ~96 % da capacidade)
// T_carga = t_pad x n x I_pad / I_carr = 4,0 h x 3 x 1625 mA / 1000 mA = 19,5 h (Eq. tempo-carga do P4)
const uint32_t T_CARGA_MS = 70200UL * 1000UL / FATOR_DEMO; // Fonte: panasonic_ncr18650b + adafruit_powerboost1000c (19,5 h = 70.200 s)

// =====================================================================================
//  PARÂMETROS DE SOFTWARE (não são parâmetros do modelo; ficam só no código —
//  plano v3, Parte 1.2: "detalhe de software; sai do texto")
// =====================================================================================
const uint32_t BAUD = 115200;           // Fonte: espressif_esp32 (taxa padrão da UART0); software
const uint32_t TIMEOUT_HTTP_MS = 15000; // Fonte: software (tempo máximo de espera por resposta HTTP)
const uint32_t TIMEOUT_WIFI_MS = 10000; // Fonte: software (tempo máximo para conectar ao Wi-Fi)
const uint32_t PISCA_MS = 500;          // Fonte: software (meio período do LED ATENÇÃO piscando)
const long FUSO_S = -3L * 3600L;        // Fonte: software (horário de Brasília, UTC-3, para as datas do BNDMET)
const long DIA_S = 86400L;              // Fonte: software (segundos em um dia)

const char *BNDMET_URL = "https://api-bndmet.decea.mil.br/v1/estacoes/D6594/fenomenos/I006"; // Fonte: decea_bndmet (estação D6594, precipitação diária I006)
const char *OWM_URL = "https://pro.openweathermap.org/data/2.5/forecast?lat=-20.1433&lon=-44.1997&units=metric&cnt="; // Fonte: openweathermap_forecast5 (Brumadinho-MG)
const char *API_LEITURAS = "/leituras"; // Fonte: plano v3, Parte 4 (POST /leituras)

// =====================================================================================
//  TIPOS E ESTADO
// =====================================================================================
enum Nivel { VERDE, AMARELO, VERMELHO };
enum EstadoBateria { BAT_NORMAL, BAT_BAIXA, BAT_CRITICA, BAT_FALHA };

struct Leitura {
  int adc = 0;
  float S = 0;
  bool sensorOk = false;
  float p48 = 0, p24 = 0, p72 = 0;
  bool p48Ok = false, p24Ok = false;
  int p48DiasNulos = 0;       // dias sem registro (null ou ausentes): P48 incompleto
  bool p72Injetado = false;
  Nivel nivel = VERDE;
};

struct Energia {
  bool rede = true;
  int vbatMv = VBAT_MAX_MV;
  EstadoBateria estado = BAT_NORMAL;
  bool recarregando = false;
  uint32_t inicioRecargaMs = 0;
};

Leitura leitura;
Energia energia;
Preferences prefs;
int lSeco = L_SECO_PADRAO;
int lSat = L_SAT_PADRAO;

// Modo de teste do comando "chuva": 0 = APIs reais; 1 = P72 injetado; 2 = sem dados de chuva;
// 3 = dados incompletos injetados ("chuva parcial <mm>": P48 = mm com 1 dia sem registro, P24 = 0)
int modoChuva = 0;
float p72Teste = 0;

uint32_t ultimoCicloMs = 0;
bool cicloForcado = true; // primeiro ciclo logo após o setup
String linhaSerial = "";

// =====================================================================================
//  TEXTOS
// =====================================================================================
const char *textoNivel(Nivel n) {
  return n == VERMELHO ? "VERMELHO" : (n == AMARELO ? "AMARELO" : "VERDE");
}

const char *textoBateria(EstadoBateria e) {
  switch (e) {
    case BAT_BAIXA: return "BAIXA";
    case BAT_CRITICA: return "CRITICA";
    case BAT_FALHA: return "FALHA";
    default: return "NORMAL";
  }
}

// =====================================================================================
//  CALIBRAÇÃO (L_seco e L_sat na memória não volátil)
// =====================================================================================
void carregarCalibracao() {
  prefs.begin("calib", true);
  lSeco = prefs.getInt("seco", L_SECO_PADRAO);
  lSat = prefs.getInt("sat", L_SAT_PADRAO);
  prefs.end();
}

void salvarCalibracao() {
  prefs.begin("calib", false);
  prefs.putInt("seco", lSeco);
  prefs.putInt("sat", lSat);
  prefs.end();
}

// =====================================================================================
//  EQUAÇÃO 1 — SATURAÇÃO RELATIVA DO SOLO
// =====================================================================================
// Leitura no extremo da escala (0 ou 4095) = circuito aberto ou em curto -> falha.
bool leituraValida(int adc) {
  return adc > 0 && adc < ADC_MAX;
}

float calcularSaturacao(int adc) {
  if (lSat == lSeco) return 0.0f;
  float s = (float)(adc - lSeco) / (float)(lSat - lSeco);
  return constrain(s, 0.0f, 1.0f);
}

void lerSaturacao() {
  leitura.adc = analogRead(PIN_HIGROMETRO);
  leitura.sensorOk = leituraValida(leitura.adc) && (lSat != lSeco);
  leitura.S = leitura.sensorOk ? calcularSaturacao(leitura.adc) : 0.0f;
}

// =====================================================================================
//  CONECTIVIDADE
// =====================================================================================
bool garantirWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  String bufferTexto = "";
  bufferTexto += "[WiFi] Conectando a " + String(WIFI_SSID) + "...\r\n";
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < TIMEOUT_WIFI_MS) {
    delay(250); // Fonte: software (espera entre tentativas)
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok) bufferTexto += "[WiFi] OK - IP " + WiFi.localIP().toString() + "\r\n";
  else bufferTexto += "[WiFi] FALHA - sem conexao.\r\n";
  Serial.print(bufferTexto);
  return ok;
}

bool relogioSincronizado() {
  return time(nullptr) > 1600000000L; // Fonte: software (qualquer data posterior a 2020 indica NTP sincronizado)
}

void sincronizarRelogio() {
  configTime(FUSO_S, 0, "pool.ntp.org");
  uint32_t t0 = millis();
  while (!relogioSincronizado() && millis() - t0 < TIMEOUT_WIFI_MS) delay(250); // Fonte: software (espera NTP)
  String bufferTexto = "";
  if (relogioSincronizado()) bufferTexto += "[NTP] OK - relogio sincronizado.\r\n";
  else bufferTexto += "[NTP] FALHA - relogio nao sincronizado.\r\n";
  Serial.print(bufferTexto);
}

String formatarData(time_t t) {
  struct tm *tm = localtime(&t);
  char buf[11];
  strftime(buf, sizeof(buf), "%Y-%m-%d", tm);
  return String(buf);
}

// GET HTTPS genérico. Devolve o corpo (só com HTTP 200) ou "" em caso de erro.
// "tag" identifica a API nas linhas do terminal. "urlExibida" é a URL impressa
// (na OWM a chave é mascarada para não aparecer nos prints do TCC).
String httpsGet(const char *tag, const String &url, const String &urlExibida, const char *chaveBndmet) {
  String bufferTexto = "";
  bufferTexto += String(tag) + " GET " + urlExibida + "\r\n";

  WiFiClientSecure cliente;
  cliente.setInsecure();
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);
  if (!http.begin(cliente, url)) {
    bufferTexto += String(tag) + " FALHA - URL invalida.\r\n";
    Serial.print(bufferTexto);
    return "";
  }
  http.addHeader("Accept", "*/*"); // igual ao Postman
  if (chaveBndmet && strlen(chaveBndmet) > 0) http.addHeader("x-api-key", chaveBndmet);

  int codigo = http.GET();
  String corpo = "";
  if (codigo == 200) {
    corpo = http.getString();
  } else if (codigo > 0) {
    bufferTexto += String(tag) + " FALHA - HTTP " + String(codigo) + ": " + http.getString().substring(0, 120) + "\r\n";
    if (codigo == 401 || codigo == 403)
      bufferTexto += String(tag) + " Dica: chave recusada. Confira a chave no secrets.h.\r\n";
  } else {
    bufferTexto += String(tag) + " FALHA - " + http.errorToString(codigo) + " (codigo " + String(codigo) + ")\r\n";
  }
  http.end();
  Serial.print(bufferTexto);
  return corpo;
}

// Texto do valor de um dia: "12.0 mm", "null (ausente)" ou "sem registro (ausente)".
String textoDia(const String &data, bool existe, bool nulo, float mm) {
  String t = data + ": ";
  if (!existe) return t + "sem registro (ausente)";
  if (nulo) return t + "null (ausente)";
  return t + String(mm, 1) + " mm";
}

// =====================================================================================
//  EQUAÇÃO 2 — P48 (BNDMET) + P24 (OpenWeatherMap)
// =====================================================================================
// P48: soma dos totais diários (I006) dos 2 últimos dias completos (anteontem e ontem).
// Com HTTP 200, a API respondeu. Dia com null (ou ausente da lista) é dado AUSENTE: fica
// fora da soma e é contado em p48DiasNulos; P48 passa a ser um mínimo (incompleto).
// P48 só fica indisponível quando a consulta falha (HTTP/conexão/JSON).
bool buscarP48(float &p48, int &diasNulos) {
  const char *tag = "[BNDMET I006]";
  if (!relogioSincronizado()) {
    String bufferTexto = "";
    bufferTexto += String(tag) + " PULADO - horario ainda nao sincronizado.\r\n";
    Serial.print(bufferTexto);
    return false;
  }
  time_t agora = time(nullptr);
  String dAnteontem = formatarData(agora - DIAS_P48 * DIA_S);
  String dOntem = formatarData(agora - DIA_S);
  String url = String(BNDMET_URL) + "?dataInicio=" + dAnteontem + "&dataFinal=" + dOntem;
  String corpo = httpsGet(tag, url, url, BNDMET_API_KEY);
  if (corpo.isEmpty()) return false;

  String bufferTexto = "";
  JsonDocument doc;
  DeserializationError erro = deserializeJson(doc, corpo);
  if (erro) {
    bufferTexto += String(tag) + " FALHA - JSON invalido: " + String(erro.c_str()) + "\r\n";
    Serial.print(bufferTexto);
    return false;
  }
  JsonArray dados = doc["data"]["data"].as<JsonArray>(); // pares [timestamp_ms, valor_ou_null]
  if (dados.isNull()) {
    bufferTexto += String(tag) + " FALHA - campo data.data ausente.\r\n";
    Serial.print(bufferTexto);
    return false;
  }

  // Associa cada par ao dia pela data do timestamp.
  bool existeA = false, nuloA = false, existeO = false, nuloO = false;
  float mmA = 0, mmO = 0;
  for (JsonArray par : dados) {
    if (par.size() < 2) continue;
    String dia = formatarData((time_t)(par[0].as<long long>() / 1000LL));
    bool nulo = par[1].isNull();
    float mm = nulo ? 0.0f : par[1].as<float>(); // null: não entra na soma (ausente)
    if (dia == dAnteontem) { existeA = true; nuloA = nulo; mmA = mm; }
    if (dia == dOntem) { existeO = true; nuloO = nulo; mmO = mm; }
  }
  diasNulos = (!existeA || nuloA) + (!existeO || nuloO);
  p48 = (existeA && !nuloA ? mmA : 0.0f) + (existeO && !nuloO ? mmO : 0.0f); // só dias com valor

  bufferTexto += String(tag) + " anteontem " + textoDia(dAnteontem, existeA, nuloA, mmA) +
                 " | ontem " + textoDia(dOntem, existeO, nuloO, mmO) + "\r\n";
  if (diasNulos == 0)
    bufferTexto += String(tag) + " OK - P48 = " + String(p48, 1) + " mm\r\n";
  else
    bufferTexto += String(tag) + " INCOMPLETO - P48 >= " + String(p48, 1) + " mm (" + String(diasNulos) +
                   " dia(s) sem registro na estacao)\r\n";
  Serial.print(bufferTexto);
  return true;
}

// P24: soma de rain.3h (mm) nos 8 próximos blocos de 3 h. Bloco sem "rain" = 0 mm.
bool buscarP24(float &p24) {
  const char *tag = "[OWM 24h]";
  String base = String(OWM_URL) + BLOCOS_P24 + "&appid=";
  String corpo = httpsGet(tag, base + OWM_API_KEY, base + "***", nullptr);
  if (corpo.isEmpty()) return false;

  String bufferTexto = "";
  JsonDocument doc;
  DeserializationError erro = deserializeJson(doc, corpo);
  if (erro) {
    bufferTexto += String(tag) + " FALHA - JSON invalido: " + String(erro.c_str()) + "\r\n";
    Serial.print(bufferTexto);
    return false;
  }
  JsonArray lista = doc["list"].as<JsonArray>();
  if (lista.isNull() || (int)lista.size() < BLOCOS_P24) {
    bufferTexto += String(tag) + " FALHA - menos de " + String(BLOCOS_P24) + " blocos de 3 h na resposta.\r\n";
    Serial.print(bufferTexto);
    return false;
  }

  float soma = 0;
  int n = 0;
  bufferTexto += String(tag) + " mm por bloco de 3 h:";
  for (JsonObject bloco : lista) {
    float mm = bloco["rain"]["3h"] | 0.0f;
    soma += mm;
    bufferTexto += String(n == 0 ? " " : " | ") + String(mm, 1);
    if (++n >= BLOCOS_P24) break;
  }
  bufferTexto += "\r\n";
  p24 = soma;
  bufferTexto += String(tag) + " OK - P24 = " + String(p24, 1) + " mm\r\n";
  Serial.print(bufferTexto);
  return true;
}

void obterChuva() {
  leitura.p72Injetado = false;
  leitura.p48DiasNulos = 0;
  if (modoChuva == 1) { // teste: P72 injetado pelo comando "chuva <mm>"
    leitura.p48Ok = leitura.p24Ok = true;
    leitura.p48 = leitura.p24 = 0;
    leitura.p72 = p72Teste;
    leitura.p72Injetado = true;
    return;
  }
  if (modoChuva == 2) { // teste: simula APIs fora do ar
    leitura.p48Ok = leitura.p24Ok = false;
    return;
  }
  if (modoChuva == 3) { // teste: dados incompletos (dia null no BNDMET); mínimo garantido = mm informado
    leitura.p48Ok = leitura.p24Ok = true;
    leitura.p48 = p72Teste;
    leitura.p24 = 0;
    leitura.p48DiasNulos = 1;
    leitura.p72 = p72Teste;
    leitura.p72Injetado = true;
    return;
  }
  bool rede = garantirWiFi();
  leitura.p48Ok = rede && buscarP48(leitura.p48, leitura.p48DiasNulos);
  leitura.p24Ok = rede && buscarP24(leitura.p24);
  if (leitura.p48Ok && leitura.p24Ok) leitura.p72 = leitura.p48 + leitura.p24; // Eq. 2
}

// Chuva completa: as duas parcelas obtidas e nenhum dia sem registro no BNDMET.
bool chuvaCompleta() {
  return leitura.p48Ok && leitura.p24Ok && leitura.p48DiasNulos == 0;
}

// Chuva incompleta: algo foi obtido, mas falta uma parcela ou um dia do BNDMET.
bool chuvaIncompleta() {
  return (leitura.p48Ok || leitura.p24Ok) && !chuvaCompleta();
}

// P72 mínimo garantido: soma do que foi obtido (a chuva ausente não é negativa).
float p72Minimo() {
  return (leitura.p48Ok ? leitura.p48 : 0.0f) + (leitura.p24Ok ? leitura.p24 : 0.0f);
}

// Chuva usada na regra: P72 completo, ou o mínimo garantido se ele já atinge P1.
// Devolve false quando a regra deve tratar como "sem dados de chuva".
bool chuvaParaRegra(float &p) {
  if (chuvaCompleta()) { p = leitura.p72; return true; }
  if (chuvaIncompleta() && p72Minimo() >= P1_MM) { p = p72Minimo(); return true; }
  return false;
}

// =====================================================================================
//  EQUAÇÃO 3 + CONTINGÊNCIAS — NÍVEL DE ALERTA
// =====================================================================================
Nivel nivelPorChuva(float p72) {
  if (p72 >= P2_MM) return VERMELHO;
  if (p72 >= P1_MM) return AMARELO;
  return VERDE;
}

Nivel avaliarNivel(const Leitura &l) {
  float p = 0;
  bool chuva = chuvaParaRegra(p);
  if (l.sensorOk && chuva) { // Eq. 3: limiar bilinear (S e P72 ao mesmo tempo)
    if (l.S >= S2 && p >= P2_MM) return VERMELHO;
    if (l.S >= S1 && p >= P1_MM) return AMARELO;
    return VERDE;
  }
  if (!l.sensorOk && chuva) return nivelPorChuva(p); // higrômetro com falha: só chuva
  if (l.sensorOk && !chuva) return (l.S >= S2) ? AMARELO : VERDE; // caso especial (Parte 2.5)
  // Sem higrômetro E sem chuva: a Eq. 3 não pode ser avaliada. Decisão do orientador
  // (03/10/2026): AMARELO por precaução, mesma lógica do caso especial (Parte 2.5).
  return AMARELO;
}

// =====================================================================================
//  ALERTA LOCAL (LEDs de nível e buzzer)
// =====================================================================================
void acionarAlertaLocal(Nivel n) {
  digitalWrite(PIN_LED_VERDE, n == VERDE);
  digitalWrite(PIN_LED_AMARELO, n == AMARELO);
  digitalWrite(PIN_LED_VERMELHO, n == VERMELHO);
  // Buzzer só no VERMELHO (Parte 4 do plano). tone() só é chamado na mudança de estado.
  static bool buzzerLigado = false;
  bool ligar = (n == VERMELHO);
  if (ligar && !buzzerLigado) tone(PIN_BUZZER, FREQ_BUZZER_HZ);
  if (!ligar && buzzerLigado) noTone(PIN_BUZZER);
  buzzerLigado = ligar;
}

// =====================================================================================
//  SUPERVISÃO DE ENERGIA
// =====================================================================================
int lerTensaoBateriaMv() {
  long adc = analogRead(PIN_BATERIA_ADC);
  return VBAT_MIN_MV + (int)(adc * (VBAT_MAX_MV - VBAT_MIN_MV) / ADC_MAX);
}

EstadoBateria classificarBateria(const Energia &e) {
  if (e.rede && e.recarregando && millis() - e.inicioRecargaMs >= T_CARGA_MS) return BAT_FALHA;
  if (e.vbatMv < VBAT_CRITICA_MV) return BAT_CRITICA;
  if (e.vbatMv < VBAT_BAIXA_MV) return BAT_BAIXA;
  return BAT_NORMAL;
}

// Devolve true quando a rede ou o estado da bateria mudou (evento de energia).
bool supervisionarEnergia() {
  bool redeAnterior = energia.rede;
  EstadoBateria estadoAnterior = energia.estado;

  energia.rede = digitalRead(PIN_REDE) == HIGH;
  energia.vbatMv = lerTensaoBateriaMv();

  // Recarga: rede presente e bateria abaixo da tensão de fim de carga.
  bool emRecarga = energia.rede && energia.vbatMv < VBAT_MAX_MV;
  if (emRecarga && !energia.recarregando) energia.inicioRecargaMs = millis();
  energia.recarregando = emRecarga;

  energia.estado = classificarBateria(energia);
  return energia.rede != redeAnterior || energia.estado != estadoAnterior;
}

void atualizarLedsEnergia() {
  digitalWrite(PIN_LED_REDE, energia.rede);
  digitalWrite(PIN_LED_BACKUP, !energia.rede);
  bool pisca = (millis() / PISCA_MS) % 2;
  switch (energia.estado) {
    case BAT_FALHA: digitalWrite(PIN_LED_ATENCAO, HIGH); break; // aceso: substituir bateria
    case BAT_BAIXA:
    case BAT_CRITICA: digitalWrite(PIN_LED_ATENCAO, pisca); break; // piscando
    default: digitalWrite(PIN_LED_ATENCAO, LOW);
  }
}

String textoEnergia() {
  String bufferTexto = "";
  bufferTexto += "[ENERGIA] rede=" + String(energia.rede ? "PRESENTE" : "AUSENTE (backup)") +
                 " | vbat=" + String(energia.vbatMv / 1000.0f, 2) + " V | bateria=" +
                 textoBateria(energia.estado) + (energia.recarregando && energia.estado != BAT_FALHA ? " (recarregando)" : "") + "\r\n";
  if (energia.recarregando && energia.estado != BAT_FALHA) {
    uint32_t decorrido = (millis() - energia.inicioRecargaMs) / 1000;
    bufferTexto += "[ENERGIA] recarga: " + String(decorrido) + " s de " + String(T_CARGA_MS / 1000) + " s ate FALHA\r\n";
  }
  return bufferTexto;
}

// =====================================================================================
//  ENVIO JSON AO BACKEND
// =====================================================================================
String montarJson(const char *evento) {
  JsonDocument doc;
  doc["evento"] = evento; // "ciclo" ou "energia"
  doc["nivel"] = textoNivel(leitura.nivel);
  if (leitura.sensorOk) doc["S"] = serialized(String(leitura.S, 2)); else doc["S"] = nullptr;
  if (leitura.p48Ok && !leitura.p72Injetado) doc["P48"] = leitura.p48; else doc["P48"] = nullptr;
  if (leitura.p24Ok && !leitura.p72Injetado) doc["P24"] = leitura.p24; else doc["P24"] = nullptr;
  if (chuvaCompleta()) doc["P72"] = leitura.p72; else doc["P72"] = nullptr;
  if (chuvaIncompleta()) doc["p72Minimo"] = p72Minimo(); // limite inferior de P72
  doc["p48DiasNulos"] = leitura.p48DiasNulos; // dias sem registro no BNDMET (P48 incompleto)
  doc["rede"] = energia.rede;
  doc["vbat"] = serialized(String(energia.vbatMv / 1000.0f, 2));
  doc["estadoBateria"] = textoBateria(energia.estado);
  doc["sensorIndisponivel"] = !leitura.sensorOk;
  doc["p48Indisponivel"] = !leitura.p48Ok;
  doc["p24Indisponivel"] = !leitura.p24Ok;
  doc["chuvaIncompleta"] = chuvaIncompleta();
  doc["semDadosChuva"] = !leitura.p48Ok && !leitura.p24Ok;
  doc["p72Injetado"] = leitura.p72Injetado; // true = cenário de teste (comando "chuva")
  doc["simulacao"] = true;
  String saida;
  serializeJson(doc, saida);
  return saida;
}

void enviarLeitura(const char *evento) {
  String json = montarJson(evento);
  String bufferTexto = "";
  bufferTexto += "[JSON] " + json + "\r\n";
  Serial.print(bufferTexto);
  if (!garantirWiFi()) return;
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);
  http.begin(String(API_BASE_URL) + API_LEITURAS);
  http.addHeader("Content-Type", "application/json");
  int codigo = http.POST(json);
  http.end();
  bufferTexto = "";
  bufferTexto += "[BACKEND] POST " + String(API_BASE_URL) + API_LEITURAS + " -> " + String(codigo) + "\r\n";
  Serial.print(bufferTexto);
}

// =====================================================================================
//  CICLO DE LEITURA (60 min / 10 min)
// =====================================================================================
uint32_t intervaloAtual() {
  bool chuva = (leitura.p48Ok && leitura.p48 > 0) || (leitura.p24Ok && leitura.p24 > 0) ||
               (leitura.p72Injetado && leitura.p72 > 0);
  return (chuva || leitura.nivel != VERDE) ? INTERVALO_EVENTO_MS : INTERVALO_NORMAL_MS;
}

String textoLeitura() {
  String bufferTexto = "";
  bufferTexto += "--------------------------------------------------\r\n";
  if (leitura.sensorOk)
    bufferTexto += "[SOLO]  ADC=" + String(leitura.adc) + " | S=" + String(leitura.S, 2) +
                   " (S1=" + String(S1, 2) + ", S2=" + String(S2, 2) + ")\r\n";
  else
    bufferTexto += "[SOLO]  ADC=" + String(leitura.adc) + " | HIGROMETRO INDISPONIVEL\r\n";

  if (leitura.p72Injetado && chuvaCompleta())
    bufferTexto += "[CHUVA] P72=" + String(leitura.p72, 1) + " mm (INJETADO - TESTE)\r\n";
  else if (chuvaCompleta())
    bufferTexto += "[CHUVA] P48=" + String(leitura.p48, 1) + " + P24=" + String(leitura.p24, 1) +
                   " = P72=" + String(leitura.p72, 1) + " mm (P1=" + String(P1_MM, 0) + ", P2=" + String(P2_MM, 0) + ")\r\n";
  else if (chuvaIncompleta()) {
    bufferTexto += "[CHUVA] DADOS DE CHUVA INCOMPLETOS - P72 >= " + String(p72Minimo(), 1) + " mm (minimo garantido)\r\n";
    bufferTexto += "[CHUVA] P48: " + String(!leitura.p48Ok ? "indisponivel" :
                   (leitura.p48DiasNulos > 0 ? String(leitura.p48, 1) + " mm, " + String(leitura.p48DiasNulos) + " dia(s) sem registro" :
                   String(leitura.p48, 1) + " mm")) +
                   " | P24: " + String(leitura.p24Ok ? String(leitura.p24, 1) + " mm" : "indisponivel") + "\r\n";
    if (p72Minimo() < P1_MM) bufferTexto += "[CHUVA] Minimo abaixo de P1: tratado como sem dados de chuva\r\n";
    if (leitura.p72Injetado) bufferTexto += "[CHUVA] (INJETADO - TESTE: dia sem registro simulado)\r\n";
  } else
    bufferTexto += "[CHUVA] SEM DADOS DE CHUVA (P48 e P24 indisponiveis)\r\n";

  bufferTexto += "[NIVEL] " + String(textoNivel(leitura.nivel)) + "\r\n";
  bufferTexto += textoEnergia();
  bufferTexto += "[CICLO] proximo em " + String(intervaloAtual() / 1000) + " s (modo demonstracao x" + String(FATOR_DEMO) + ")\r\n";
  return bufferTexto;
}

void executarCiclo() {
  lerSaturacao();
  obterChuva();
  leitura.nivel = avaliarNivel(leitura);
  acionarAlertaLocal(leitura.nivel);
  Serial.print(textoLeitura());
  enviarLeitura("ciclo");
  ultimoCicloMs = millis();
  cicloForcado = false;
}

// =====================================================================================
//  COMANDOS SERIAL (somente teste)
// =====================================================================================
void cmdHelp() {
  String bufferTexto = "";
  bufferTexto += "Comandos:\r\n";
  bufferTexto += "  status          - leitura atual do solo, chuva, nivel e energia\r\n";
  bufferTexto += "  calibrar        - mostra L_seco e L_sat\r\n";
  bufferTexto += "  calibrar seco   - grava a leitura atual como L_seco (solo seco)\r\n";
  bufferTexto += "  calibrar sat    - grava a leitura atual como L_sat (solo saturado)\r\n";
  bufferTexto += "  calibrar reset  - volta para 0 / 4095\r\n";
  bufferTexto += "  chuva <mm>      - TESTE: injeta P72 em mm e roda um ciclo\r\n";
  bufferTexto += "  chuva parcial <mm> - TESTE: dados incompletos (1 dia sem registro); minimo = mm\r\n";
  bufferTexto += "  chuva na        - TESTE: simula APIs fora do ar (sem dados de chuva)\r\n";
  bufferTexto += "  chuva off       - volta a usar BNDMET + OpenWeatherMap\r\n";
  bufferTexto += "  help            - esta lista\r\n";
  Serial.print(bufferTexto);
}

String textoModoChuva() {
  return modoChuva == 1 ? "injetado" : (modoChuva == 2 ? "sem dados" : (modoChuva == 3 ? "incompleto injetado" : "APIs"));
}

void cmdStatus() {
  String bufferTexto = textoLeitura(); // valores do último ciclo
  int adc = analogRead(PIN_HIGROMETRO); // potenciômetro agora (o nível só muda no próximo ciclo)
  bufferTexto += "[AGORA] ADC=" + String(adc) + " | S=" + String(calcularSaturacao(adc), 2) +
                 (leituraValida(adc) ? "" : " (leitura invalida)") + "\r\n";
  bufferTexto += "[CALIB] L_seco=" + String(lSeco) + " | L_sat=" + String(lSat) + " | modo chuva=" + textoModoChuva() + "\r\n";
  Serial.print(bufferTexto);
}

void cmdCalibrar(const String &arg) {
  int adc = analogRead(PIN_HIGROMETRO);
  if (arg == "seco") lSeco = adc;
  else if (arg == "sat") lSat = adc;
  else if (arg == "reset") { lSeco = L_SECO_PADRAO; lSat = L_SAT_PADRAO; }
  if (arg.length()) salvarCalibracao();
  String bufferTexto = "";
  bufferTexto += "[CALIB] L_seco=" + String(lSeco) + " | L_sat=" + String(lSat) + " | ADC atual=" + String(adc) + "\r\n";
  if (lSeco == lSat) bufferTexto += "[CALIB] ATENCAO: L_seco = L_sat, a Eq. 1 nao pode ser calculada\r\n";
  Serial.print(bufferTexto);
}

void cmdChuva(const String &arg) {
  String bufferTexto = "";
  if (arg == "off") modoChuva = 0;
  else if (arg == "na") modoChuva = 2;
  else if (arg.startsWith("parcial ") && arg.length() > 8 && (isDigit(arg[8]) || arg[8] == '.')) {
    modoChuva = 3;
    p72Teste = arg.substring(8).toFloat();
  }
  else if (arg.length() && (isDigit(arg[0]) || arg[0] == '.')) { modoChuva = 1; p72Teste = arg.toFloat(); }
  else {
    bufferTexto += "Uso: chuva <mm> | chuva parcial <mm> | chuva na | chuva off\r\n";
    Serial.print(bufferTexto);
    return;
  }
  bufferTexto += "[TESTE] modo chuva = " + textoModoChuva() + "\r\n";
  Serial.print(bufferTexto);
  cicloForcado = true;
}

void tratarComando(String linha) {
  linha.trim();
  linha.toLowerCase();
  int esp = linha.indexOf(' ');
  String cmd = esp < 0 ? linha : linha.substring(0, esp);
  String arg = esp < 0 ? "" : linha.substring(esp + 1);
  arg.trim();
  if (cmd == "status") cmdStatus();
  else if (cmd == "calibrar") cmdCalibrar(arg);
  else if (cmd == "chuva") cmdChuva(arg);
  else if (cmd == "help") cmdHelp();
  else if (cmd.length()) {
    String bufferTexto = "";
    bufferTexto += "Comando desconhecido. Digite help.\r\n";
    Serial.print(bufferTexto);
  }
}

void lerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (linhaSerial.length()) tratarComando(linhaSerial);
      linhaSerial = "";
    } else {
      linhaSerial += c;
    }
  }
}

// =====================================================================================
//  SETUP E LOOP
// =====================================================================================
void configurarPinos() {
  const int saidas[] = {PIN_LED_VERDE, PIN_LED_AMARELO, PIN_LED_VERMELHO, PIN_BUZZER,
                        PIN_LED_REDE, PIN_LED_BACKUP, PIN_LED_ATENCAO};
  for (int p : saidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(PIN_REDE, INPUT);
  analogReadResolution(12); // Fonte: espressif_esp32 (ADC de 12 bits)
}

void setup() {
  Serial.begin(BAUD);
  delay(500); // Fonte: software (estabiliza a serial)
  String bufferTexto = "";
  bufferTexto += "\r\n=== Alerta de barragem v3 (Wokwi / ESP32) - SIMULACAO ===\r\n";
  bufferTexto += "Modo demonstracao: tempos divididos por " + String(FATOR_DEMO) + "\r\n";
  Serial.print(bufferTexto);
  configurarPinos();
  carregarCalibracao();
  supervisionarEnergia();
  atualizarLedsEnergia();
  if (garantirWiFi()) sincronizarRelogio();
  cmdHelp();
}

void loop() {
  lerSerial();
  if (supervisionarEnergia()) { // evento de energia: informa na hora
    Serial.print(textoEnergia());
    enviarLeitura("energia");
  }
  atualizarLedsEnergia();
  if (cicloForcado || millis() - ultimoCicloMs >= intervaloAtual()) executarCiclo();
  delay(20); // Fonte: software (alivia a CPU do simulador)
}
