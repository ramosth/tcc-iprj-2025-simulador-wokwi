/*
 * ========================================================================================================================
 *  SISTEMA DE MONITORAMENTO DE BARRAGEM DE REJEITOS — SIMULAÇÃO WOKWI
 *  ESP32 DevKit | TCC UERJ-IPRJ | Thamires Ramos dos Santos
 *
 *  VERSÃO 4 — teste de escalada natural via chuva sintética
 *
 *  CHANGELOG v4 (sobre a v3):
 *  [NOVO] Comando serial "chuva sintetica" / "chuva real" — toggle em
 *    runtime (não é #define, não exige recompilar/reflashear) que fixa os
 *    quatro componentes pluviométricos (precip24h/7d/30d + previsao24h) no
 *    teto de seus respectivos limiares, enquanto V_lencol, V_taxa e
 *    V_pressao continuam vindos do sensor/histórico REAIS. Objetivo: com a
 *    flag ativa, basta variar a umidade real do potenciômetro para
 *    atravessar VERDE -> AMARELO -> VERMELHO pela soma ponderada da
 *    Equação 5 (incluindo o mecanismo de amplificação x1,20 disparando ao
 *    cruzar 17,5% de umidade), sem depender de chuva real nem do protocolo
 *    de ruptura por limiar de 30%. Quando a flag está em false (padrão de
 *    boot), o bloco inteiro é ignorado por um único "if" em
 *    analisarRiscoIntegrado() e o fluxo real permanece bit-a-bit idêntico
 *    ao da v3 — nenhuma outra função foi alterada.
 *  [IMPORTANTE] dadosBrutos.chuva_sintetica_ativa adicionado ao payload,
 *    para que todo registro gerado com a flag ativa fique identificável no
 *    banco e nunca seja confundido com uma leitura real de BNDMET/OWM.
 *
 *  VERSÃO 3 — fidelidade com tcc_versao_19.ino (ESP8266, produção)
 *
 *  CHANGELOG v3 (sobre a v2):
 *  [CRÍTICO] Intervalos de sensor, BNDMET e envio à API voltam a ser
 *    ADAPTATIVOS por nível de alerta (statusSistema), via
 *    obterIntervaloSensor()/obterIntervaloBNDMET()/obterIntervaloEnvioAPI(),
 *    replicando exatamente os valores do tcc_versao_19.ino:
 *      sensor:  30s/10s/5s (VERDE/AMARELO/VERMELHO)
 *      BNDMET:  5min/2min/1min
 *      envio:   60s/20s/10s
 *    Antes, esses três temporizadores eram fixos (15s/10min/20s) independente
 *    do nível de risco — confirmado nos dados de 2026-09-16 (cadência ~20s
 *    constante mesmo em VERMELHO). OWM (/weather+/forecast) permanece fixo
 *    em 10min, igual ao comportamento do v19 (não é adaptativo por design).
 *  [CRÍTICO] dadosLocais.sensorOK deixa de ser fixo em true e passa a ser
 *    calculado pelos limites do ADC (curto-circuito=0 / desconexão=4095),
 *    igual à lógica do tcc_versao_19.ino. Antes, sensor_ok era sempre 1 no
 *    banco, mesmo em leituras nos extremos do potenciômetro, impedindo o
 *    desconto de -40 na confiabilidade e mascarando cenários de falha.
 *
 *  CHANGELOG v2 (sobre a v1 — correções de compilação/boot/NTP):
 *
 *  [CRÍTICO] Corrigido parsing do BNDMET I006/I175: a API retorna pares
 *    [timestamp_ms, valor] por medição, não valores soltos. O código anterior
 *    fazia dataArr[i].as<float>() diretamente sobre o par — o ArduinoJson
 *    converte um JsonArray não-escalar para float como 0, então precipitacao24h/
 *    7d/30d/Atual ficavam sempre 0, mesmo com a API respondendo HTTP 200.
 *  [CRÍTICO] calcularTaxaVariacao() e calcularQuedaPressao() agora calculam de
 *    fato a partir dos buffers bufferUmidade[]/historicoPressao[], em vez de
 *    retornar valores fixos (0,02 / 1,1) — os dois componentes correspondentes
 *    da Equação 5 do TCC (V_taxa_var, V_pressão) voltam a refletir os dados reais.
 *  [CRÍTICO] obterIntensidadePrevisao()/calcularFatorPrevisaoIntensidade():
 *    restauradas as 5 categorias da Tabela 4 do TCC (Fraca/Moderada/Forte/
 *    Muito Forte/Pancada de Chuva — fatores 0,00/0,25/0,50/0,75/1,00), em vez
 *    das 2 categorias que existiam antes.
 *  [CRÍTICO] calcularConfiabilidadeAnalise() e gerarRecomendacaoDetalhada()
 *    implementadas e chamadas a cada análise — analiseRisco.recomendacao e
 *    .confiabilidade deixam de ficar sempre vazios/zerados (o frontend
 *    classifica ruptura via recomendacao.includes('RUPTURA')).
 *  [CRÍTICO] Payload de enviarDadosParaAPI() expandido de 7 para ~30 campos
 *    (timestamp, recomendacao, confiabilidade, os 7 componentes individuais
 *    da equação, dados do BNDMET/OWM completos, dadosBrutos com uptime/
 *    freeHeap/rssi, etc.) — igual ao criarPayloadJSON() do v19.
 *
 *  [IMPORTANTE] Comandos "alerta verde/amarelo/vermelho" agora enviam os
 *    dados simulados à API (3 envios por simulação, via simulacaoEnviosRestantes),
 *    e bloqueiam recálculo/sobrescrita automática enquanto ativos (Opção C do v19).
 *  [IMPORTANTE] historicoPressao[] agora é populado em buscarWeatherAtual()
 *    (antes ficava sempre zerado, mesmo sendo usado por calcularQuedaPressao()).
 *  [IMPORTANTE] Cooldown de 3 leituras seguras após ruptura antes de recalcular
 *    o risco normalmente (contagemRetornoRuptura / aguardandoResetRuptura) —
 *    evita alternância brusca de estado assim que a umidade cruza o limiar.
 *  [IMPORTANTE] Histórico de tendência (historicoUmidade/Precipitacao/Risco[])
 *    via atualizarHistoricoAnalise().
 *  [IMPORTANTE] Watchdog leve de conectividade: verificarConectividade(),
 *    garantirWiFi(), contador de tentativas de reconexão.
 *  [IMPORTANTE] Timestamp real (obterTimestampISO()) incluído no payload.
 *  [IMPORTANTE] Comandos de debug/diagnóstico: debug, calibrar, reset, teste,
 *    enviar, api (além de status, analise, help, alerta *).
 *
 *  Hardware simulado (Wokwi):
 *    Potenciômetro → GPIO34 (simula higrômetro capacitivo, leitura linear 0–4095)
 *    LED Verde → GPIO18 | LED Amarelo → GPIO14 | LED Vermelho → GPIO27 | Buzzer → GPIO26
 *
 *  Faixas de alerta de risco (Tabela 5 TCC):
 *    0,00 – 0,45 → VERDE   |   0,45 – 0,75 → AMARELO   |   > 0,75 → VERMELHO + Buzzer
 *    ≥ 30% umidade → RUPTURA (fatorRisco = 1,0 imediato)
 *
 *  Comandos Serial:
 *    status | analise | debug | calibrar | reset | teste | enviar | api | help
 *    alerta verde | alerta amarelo | alerta vermelho (simulação de nível)
 * ========================================================================================================================
 */

// ============================================================
//  BIBLIOTECAS
// ============================================================
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <time.h>

// ============================================================
//  CONFIGURAÇÕES
// ============================================================

// WiFi — rede virtual do simulador Wokwi (acesso à internet real via gateway do Wokwi)
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""

// API própria (backend Node.js) — IP do gateway virtual do Wokwi apontando pro host
#define API_BASE_URL "http://172.29.224.1:3001"
#define API_ENDPOINT "/api/sensor/dados"
#define API_STATUS_ENDPOINT "/api/sensor/status"
#define API_TOKEN ""

// BNDMET — api-bndmet.decea.mil.br (HTTPS)
#define BNDMET_HOST "api-bndmet.decea.mil.br"
#define BNDMET_API_KEY "F9prvVKpaQ1qNtQywCN2sily029xgNaq"
#define BNDMET_ESTACAO "D6594"
#define BNDMET_COD_I006 "I006"
#define BNDMET_COD_I175 "I175"

// OpenWeatherMap — pro.openweathermap.org (HTTPS obrigatório)
#define OWM_HOST "pro.openweathermap.org"
#define OWM_API_KEY "04b8a531e11670b8099c49e16ba8f676"
#define OWM_LAT "-20.1433"
#define OWM_LON "-44.1997"

// ============================================================
//  PINOS (ESP32 DevKit — Wokwi)
// ============================================================
#define PIN_HIGROMETRO 34
#define PIN_LED_VERDE 18
#define PIN_LED_AMARELO 14
#define PIN_LED_VERMELHO 27
#define PIN_BUZZER 26

// ============================================================
//  CALIBRAÇÃO DO SENSOR
//  Potenciômetro do Wokwi é linear 0–4095, sem inversão — diferente do
//  higrômetro capacitivo real (ESP8266, 0–1023 invertido: seco=1023, úmido=240).
// ============================================================
int SENSOR_SECO = 0;
int SENSOR_UMIDO = 4095;

// ============================================================
//  LIMIARES — FONTES VERIFICADAS (idênticos ao v19)
// ============================================================
const float UMIDADE_CRITICA = 25.0f;
const float UMIDADE_RUPTURA = 30.0f;

const float LIMIAR_24H = 50.0f;
const float LIMIAR_7D = 150.0f;
const float LIMIAR_30D = 300.0f;
const float LIMIAR_24H_OWM = 80.0f;

const float QUEDA_PRESSAO_ALERTA = 5.0f;
const float PRESSAO_MIN = 700.0f;
const float PRESSAO_MAX = 1100.0f;

// ============================================================
//  PESOS DA FÓRMULA (soma = 1,00)
// ============================================================
const float PESO_LENCOL = 0.40f;
const float PESO_CH_ATUAL = 0.08f;
const float PESO_CH_HIST = 0.12f;
const float PESO_CH_MENSAL = 0.10f;
const float PESO_CH_FUTURA = 0.15f;
const float PESO_TAXA_VAR = 0.10f;
const float PESO_PRESSAO = 0.05f;

const float FATOR_AMPLIF = 1.20f;
const float LIMIAR_SOLO_SAT = 0.70f;

// Faixas de alerta (Tabela 5 TCC)
const float LIMIAR_VERDE = 0.45f;
const float LIMIAR_AMARELO = 0.75f;

// ============================================================
//  BUFFERS E HISTÓRICO
// ============================================================
const int BUFFER_UMIDADE = 10;
const int HIST_PRESSAO = 6;

// ============================================================
//  INTERVALOS (ms) — ADAPTATIVOS POR NÍVEL DE ALERTA
//  [CRÍTICO — corrigido] Restaurada a lógica adaptativa por statusSistema
//  (0=VERDE | 1=AMARELO | 2=VERMELHO), idêntica ao tcc_versao_19.ino.
//  As funções obterIntervaloSensor()/obterIntervaloBNDMET()/obterIntervaloEnvioAPI()
//  abaixo substituem o uso direto das antigas constantes fixas no loop().
// ============================================================
const unsigned long INTERVALO_SENSOR_VERDE = 30000UL;   // leitura do potenciômetro — 30 s (VERDE)
const unsigned long INTERVALO_SENSOR_AMARELO = 10000UL; // 10 s (AMARELO)
const unsigned long INTERVALO_SENSOR_VERMELHO = 5000UL; // 5 s (VERMELHO)

// [AJUSTE] INTERVALO_ANALISE fixo removido: a análise de risco periódica
// (mais abaixo, no loop()) agora usa obterIntervaloSensor() em vez de um
// valor constante — assim ela acompanha o mesmo ritmo adaptativo da leitura
// do sensor (30s/10s/5s conforme o nível de alerta), em vez de rodar sempre
// a cada 10s independentemente do nível atual.
// const unsigned long INTERVALO_ANALISE = 10000UL;

const unsigned long INTERVALO_ENVIO_API_VERDE = 60000UL;    // envio ao backend — 60 s (VERDE)
const unsigned long INTERVALO_ENVIO_API_AMARELO = 20000UL;  // 20 s (AMARELO)
const unsigned long INTERVALO_ENVIO_API_VERMELHO = 10000UL; // 10 s (VERMELHO)

const unsigned long INTERVALO_BNDMET_VERDE = 300000UL;   // consulta BNDMET — 5 min (VERDE)
const unsigned long INTERVALO_BNDMET_AMARELO = 120000UL; // 2 min (AMARELO)
const unsigned long INTERVALO_BNDMET_VERMELHO = 60000UL; // 1 min (VERMELHO)

const unsigned long INTERVALO_METEO = 600000UL; // OWM /weather+/forecast — 10 min fixos (independente do nível, igual ao v19)

const unsigned long INTERVALO_RETRY_API_FALHA = 30000UL; // retry acelerado após falha BNDMET/OWM

// statusSistema é definida mais abaixo (seção VARIÁVEIS GLOBAIS); declaração
// antecipada necessária pois as funções de intervalo a referenciam antes disso.
extern int statusSistema; // 0=Verde | 1=Amarelo | 2=Vermelho

// Funções que resolvem o intervalo vigente conforme o nível de alerta atual (statusSistema)
unsigned long obterIntervaloSensor()
{
  switch (statusSistema)
  {
  case 2:
    return INTERVALO_SENSOR_VERMELHO;
  case 1:
    return INTERVALO_SENSOR_AMARELO;
  default:
    return INTERVALO_SENSOR_VERDE;
  }
}

unsigned long obterIntervaloEnvioAPI()
{
  switch (statusSistema)
  {
  case 2:
    return INTERVALO_ENVIO_API_VERMELHO;
  case 1:
    return INTERVALO_ENVIO_API_AMARELO;
  default:
    return INTERVALO_ENVIO_API_VERDE;
  }
}

unsigned long obterIntervaloBNDMET()
{
  switch (statusSistema)
  {
  case 2:
    return INTERVALO_BNDMET_VERMELHO;
  case 1:
    return INTERVALO_BNDMET_AMARELO;
  default:
    return INTERVALO_BNDMET_VERDE;
  }
}

// ============================================================
//  ESTRUTURAS DE DADOS
// ============================================================
struct DadosLocais
{
  int valorADC;
  float umidadeSolo;
  float fatorLocal;
  bool sensorOK;
  unsigned long timestamp;
};

struct DadosBNDMET
{
  float precipitacao24h;
  float precipitacao7d;
  float precipitacao30d;
  float precipitacaoAtual; // I175 — medição horária mais próxima do instante atual
  int qualidadeDados;      // % de medições válidas (não nulas) retornadas pela API
  String statusAPI;        // "OK" | "FALHA" | "PENDENTE"
  String estacao;
  unsigned long timestamp;
  bool apiDisponivel;
};

struct DadosMeteorologicos
{
  float temperatura;
  float umidadeExterna;
  float pressaoAtmosferica;
  float velocidadeVento;
  float chuvaAtualOWM;
  String descricaoTempo;
  unsigned long timestamp;
};

struct PrevisaoTempo
{
  float chuvaFutura24h;
  String intensidadePrevisao;
  float fatorIntensidade;
  unsigned long timestamp;
};

struct AnaliseRisco
{
  float riscoIntegrado;
  int indiceRisco;
  String nivelAlerta; // "VERDE" | "AMARELO" | "VERMELHO"
  String statusTexto; // "SEGURO" | "ATENÇÃO" | "CRÍTICO" | "RUPTURA"
  String recomendacao;
  int confiabilidade;
  bool amplificado;

  // Componentes individuais da Equação 5 TCC expandida (cada um já com o peso aplicado)
  float vLencol;
  float vChuvaAtual;
  float vChuvaHistorica;
  float vChuvaMensal;
  float vChuvaFutura;
  float vTaxaVariacao;
  float vPressao;
};

// Detalhes do cálculo de confiabilidade (log/depuração)
struct ConfiabilidadeDetalhes
{
  int descontoSensor;
  int descontoBndmetFora;
  int descontoQualidadeBndmet;
  int descontoOWM;
  int descontoWifi;
  int descontoBuffer;
  int totalDesconto;
  int resultado;
  bool estadoEspecialRuptura;
};

// ============================================================
//  VARIÁVEIS GLOBAIS
// ============================================================
DadosLocais dadosLocais;
DadosBNDMET dadosBNDMET;
DadosMeteorologicos dadosMeteo;
PrevisaoTempo previsao;
AnaliseRisco analiseRisco;
ConfiabilidadeDetalhes detalhesConfiab;

// Buffers circulares
float bufferUmidade[BUFFER_UMIDADE] = {0};
int bufferIndex = 0;
bool bufferCheio = false;

float historicoPressao[HIST_PRESSAO] = {0};
int indexPressao = 0;
int totalLeiturasPressao = 0;

// Histórico de tendência (últimas 10 análises)
float historicoUmidade[10] = {0};
float historicoPrecipitacao[10] = {0};
float historicoRisco[10] = {0};
int indiceHistorico = 0;

// Controle de estado
bool wifiConectado = false;
bool apiConectada = false;
bool modoManual = false;
bool simulacaoAtiva = false;
// [TESTE] Flag de runtime (não altera compilação) para sobrepor a chuva por
// valores fixos de evento crítico, permitindo demonstrar a escalada NATURAL
// da equação (incluindo o mecanismo de amplificação) variando apenas a
// umidade real do sensor. Controlada pelos comandos Serial "chuva sintetica"
// / "chuva real". Quando false (padrão), nenhuma linha deste recurso executa
// e o fluxo real permanece 100% inalterado.
bool chuvaSinteticaAtiva = false;
int simulacaoEnviosRestantes = 0;
bool buzzerAtivo = false;
int statusSistema = 0; // 0=Verde | 1=Amarelo | 2=Vermelho
int tentativasEnvioAPI = 0;
int tentativasReconexao = 0;
int totalLeiturasSensor = 0;
bool aguardandoResetRuptura = false; // true enquanto em cooldown de retorno de ruptura (<3 leituras seguras)
bool bndmetInicializado = false;     // true assim que o NTP sincroniza e a 1a tentativa de BNDMET é feita

// Timestamps de controle (ms desde o boot)
unsigned long ultimaLeituraSensor = 0;
unsigned long ultimaLeituraBNDMET = 0;
unsigned long ultimaOWM = 0;
unsigned long ultimaAnalise = 0;
unsigned long ultimoEnvioAPI = 0;

// ============================================================
//  PROTÓTIPOS
// ============================================================
void analisarRiscoIntegrado();
void aplicarLimitesAlerta();
void acionarRuptura();
void controlarSistemaFisico();
void ativarAlarmeBuzzer();
float calcularFatorLencolFreatico(float umidade);
float calcularVLencolFreatico(float umidade);
float calcularVChuvaAtual(float precip24h);
float calcularVChuvaHistorica(float precip7d);
float calcularVChuvaMensal(float precip30d);
float calcularVChuvaFutura(float chuvaFutura24h);
float calcularTaxaVariacao();
float calcularQuedaPressao();
String obtenerFormatData(time_t t);
String obterTimestampISO();
String obterIntensidadePrevisao();
float calcularFatorPrevisaoIntensidade(String intensidade);
void buscarDadosDiarios_I006();
void buscarDadosHorarios_I175();
bool buscarWeatherAtual();
bool buscarForecast();
void conectarSistemas();
void garantirWiFi();
void verificarConectividade();
bool verificarStatusAPI();
bool enviarDadosParaAPI();
String criarPayloadJSON();
void calcularConfiabilidadeAnalise();
void gerarRecomendacaoDetalhada();
void atualizarHistoricoAnalise();
void mostrarStatusConectividade();
void mostrarAnaliseDetalhada();
void debugConectividade();
void recalibrarSensor();
void executarTesteCompleto();
void testarCalculosTCC();
void resetarSistema();
void mostrarComandosDisponiveis();

// ============================================================
//  ANÁLISE DE RISCO — EQUAÇÃO 5 TCC
// ============================================================
float calcularFatorLencolFreatico(float umidade)
{
  return constrain(umidade / UMIDADE_CRITICA, 0.0f, 1.0f);
}

float calcularVLencolFreatico(float umidade)
{
  return calcularFatorLencolFreatico(umidade) * PESO_LENCOL;
}

float calcularVChuvaAtual(float precip24h)
{
  return constrain(precip24h / LIMIAR_24H, 0.0f, 1.0f) * PESO_CH_ATUAL;
}

float calcularVChuvaHistorica(float precip7d)
{
  return constrain(precip7d / LIMIAR_7D, 0.0f, 1.0f) * PESO_CH_HIST;
}

float calcularVChuvaMensal(float precip30d)
{
  return constrain(precip30d / LIMIAR_30D, 0.0f, 1.0f) * PESO_CH_MENSAL;
}

float calcularVChuvaFutura(float chuvaFutura24h)
{
  String intensidade = obterIntensidadePrevisao();
  return calcularFatorPrevisaoIntensidade(intensidade) * PESO_CH_FUTURA;
}

// [CRÍTICO — corrigido] Taxa de variação real, calculada a partir do buffer
// circular de umidade (bufferUmidade[]), em vez de retornar 0,02 fixo.
// Compara a leitura mais recente com a mais antiga disponível no buffer.
float calcularTaxaVariacao()
{
  int leituras = bufferCheio ? BUFFER_UMIDADE : bufferIndex;
  if (leituras < 2)
    return 0.0f;

  int idxMaisRecente = (bufferIndex - 1 + BUFFER_UMIDADE) % BUFFER_UMIDADE;
  int idxMaisAntigo = bufferCheio ? bufferIndex : 0;

  float variacao = bufferUmidade[idxMaisRecente] - bufferUmidade[idxMaisAntigo];
  return constrain(variacao / 100.0f, -1.0f, 1.0f);
}

// [CRÍTICO — corrigido] Queda de pressão real, calculada a partir do
// histórico de pressão (historicoPressao[]), em vez de retornar 1,1 fixo.
// Compara a leitura mais recente com a mais antiga na janela de 3h (6 leituras × 30min).
float calcularQuedaPressao()
{
  if (totalLeiturasPressao < 2)
    return 0.0f;

  int idxMaisRecente = (indexPressao - 1 + HIST_PRESSAO) % HIST_PRESSAO;
  int idxMaisAntigo = (totalLeiturasPressao < HIST_PRESSAO)
                          ? 0
                          : (indexPressao % HIST_PRESSAO);

  float queda = historicoPressao[idxMaisAntigo] - historicoPressao[idxMaisRecente];
  return queda > 0.0f ? queda : 0.0f;
}

// [CRÍTICO — restaurado] Tabela 4 do TCC — 5 categorias de intensidade
// pluviométrica prevista, com fatores discretos 0,00/0,25/0,50/0,75/1,00.
String obterIntensidadePrevisao()
{
  float fc = previsao.chuvaFutura24h;
  if (fc >= 80.0f)
    return "Pancada de Chuva";
  if (fc >= 50.0f)
    return "Muito Forte";
  if (fc >= 25.0f)
    return "Forte";
  if (fc >= 5.0f)
    return "Moderada";
  return "Fraca";
}

float calcularFatorPrevisaoIntensidade(String intensidade)
{
  if (intensidade == "Pancada de Chuva")
    return 1.00f;
  if (intensidade == "Muito Forte")
    return 0.75f;
  if (intensidade == "Forte")
    return 0.50f;
  if (intensidade == "Moderada")
    return 0.25f;
  return 0.00f; // "Fraca"
}

void analisarRiscoIntegrado()
{
  String bufferTexto = "";
  bufferTexto += "🔍 ========== ANÁLISE DE RISCO INTEGRADO ==========\r\n";

  // [TESTE] Sobreposição opcional de chuva por comando serial. Guardada por
  // if(chuvaSinteticaAtiva) — quando a flag está em false (padrão), este
  // bloco inteiro é ignorado e o fluxo real (BNDMET/OWM/higrômetro) segue
  // sem qualquer alteração. Só V_len, V_taxa e V_press continuam vindo dos
  // dados reais (sensor + histórico de pressão), permitindo que a umidade
  // real do sensor, sozinha, atravesse VERDE -> AMARELO -> VERMELHO pela
  // equação natural (com amplificação incluída) sem depender de chuva real.
  if (chuvaSinteticaAtiva)
  {
    dadosBNDMET.precipitacao24h = 40.0f;  // > LIMIAR_24H (40mm)
    dadosBNDMET.precipitacao7d = 100.0f;  // > LIMIAR_7D  (150mm)
    dadosBNDMET.precipitacao30d = 200.0f; // > LIMIAR_30D (300mm)
    dadosBNDMET.apiDisponivel = true;     // evita desconto de confiabilidade durante o teste
    previsao.chuvaFutura24h = 10.0f;      // >= 10mm -> classe "Moderada"
    previsao.intensidadePrevisao = obterIntensidadePrevisao();
    previsao.fatorIntensidade = calcularFatorPrevisaoIntensidade(previsao.intensidadePrevisao);
    previsao.timestamp = millis(); // evita desconto de OWM indisponível durante o teste
    bufferTexto += "  🌧️ [TESTE] Chuva sintetica ativa — soma fixa dos 4 componentes pluviométricos = 0,45\r\n";
  }

  analiseRisco.vLencol = calcularVLencolFreatico(dadosLocais.umidadeSolo);
  analiseRisco.vChuvaAtual = calcularVChuvaAtual(dadosBNDMET.precipitacao24h);
  analiseRisco.vChuvaHistorica = calcularVChuvaHistorica(dadosBNDMET.precipitacao7d);
  analiseRisco.vChuvaMensal = calcularVChuvaMensal(dadosBNDMET.precipitacao30d);
  analiseRisco.vChuvaFutura = calcularVChuvaFutura(previsao.chuvaFutura24h);
  analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;
  analiseRisco.vPressao = constrain(calcularQuedaPressao() / QUEDA_PRESSAO_ALERTA, 0.0f, 1.0f) * PESO_PRESSAO;

  float soma = analiseRisco.vLencol + analiseRisco.vChuvaAtual + analiseRisco.vChuvaHistorica + analiseRisco.vChuvaMensal + analiseRisco.vChuvaFutura + analiseRisco.vTaxaVariacao + analiseRisco.vPressao;

  // Amplificação: Fator_lençol >= 0,70 E previsão >= Moderada (>=5mm/24h)
  float fatorLencol = calcularFatorLencolFreatico(dadosLocais.umidadeSolo);
  bool previsaoModeradaOuSuperior = (previsao.chuvaFutura24h >= 5.0f);
  analiseRisco.amplificado = (fatorLencol >= LIMIAR_SOLO_SAT && previsaoModeradaOuSuperior);
  if (analiseRisco.amplificado)
  {
    soma *= FATOR_AMPLIF;
    bufferTexto += "  ⚠️ AMPLIFICAÇÃO 1,2× aplicada (solo saturado + previsão ≥ Moderada)\r\n";
  }

  analiseRisco.riscoIntegrado = constrain(soma, 0.0f, 1.0f);
  analiseRisco.indiceRisco = (int)roundf(analiseRisco.riscoIntegrado * 100.0f);

  bufferTexto += "📊 Componentes (Equação 5 TCC expandida):\r\n";
  bufferTexto += "  V_lencol       = " + String(analiseRisco.vLencol, 3) + " (fator: " + String(fatorLencol, 3) + " x peso: 0,40)\r\n";
  bufferTexto += "  V_ch.atual     = " + String(analiseRisco.vChuvaAtual, 3) + " (precip24h=" + String(dadosBNDMET.precipitacao24h, 2) + "mm x 0,08)\r\n";
  bufferTexto += "  V_ch.historica = " + String(analiseRisco.vChuvaHistorica, 3) + " (precip7d=" + String(dadosBNDMET.precipitacao7d, 2) + "mm x 0,12)\r\n";
  bufferTexto += "  V_ch.mensal    = " + String(analiseRisco.vChuvaMensal, 3) + " (precip30d=" + String(dadosBNDMET.precipitacao30d, 2) + "mm x 0,10)\r\n";
  bufferTexto += "  V_ch.futura    = " + String(analiseRisco.vChuvaFutura, 3) + " (forecast24h=" + String(previsao.chuvaFutura24h, 2) + "mm, intensidade=" + previsao.intensidadePrevisao + ", fator=" + String(previsao.fatorIntensidade, 2) + " x 0,15)\r\n";
  bufferTexto += "  V_taxa_var     = " + String(analiseRisco.vTaxaVariacao, 3) + " (variacao=" + String(calcularTaxaVariacao() * 100.0f, 2) + "%)\r\n";
  bufferTexto += "  V_pressao      = " + String(analiseRisco.vPressao, 3) + " (queda=" + String(calcularQuedaPressao(), 2) + "hPa x 0,05)\r\n";
  bufferTexto += "  FATOR_RISCO TOTAL = " + String(analiseRisco.riscoIntegrado, 3) + " | INDICE = " + String(analiseRisco.indiceRisco) + "%\r\n";

  Serial.print(bufferTexto);

  aplicarLimitesAlerta();
  calcularConfiabilidadeAnalise();
  atualizarHistoricoAnalise();
  gerarRecomendacaoDetalhada();
  controlarSistemaFisico();
}

void aplicarLimitesAlerta()
{
  if (analiseRisco.riscoIntegrado <= LIMIAR_VERDE)
  {
    statusSistema = 0;
    analiseRisco.statusTexto = "SEGURO";
    analiseRisco.nivelAlerta = "VERDE";
  }
  else if (analiseRisco.riscoIntegrado <= LIMIAR_AMARELO)
  {
    statusSistema = 1;
    analiseRisco.statusTexto = "ATENÇÃO";
    analiseRisco.nivelAlerta = "AMARELO";
  }
  else
  {
    statusSistema = 2;
    analiseRisco.statusTexto = "CRÍTICO";
    analiseRisco.nivelAlerta = "VERMELHO";
  }
  String bufferTexto = "";
  bufferTexto += "🚨 STATUS: " + analiseRisco.statusTexto + " | Fator: " + String(analiseRisco.riscoIntegrado, 3) + "\r\n";

  Serial.print(bufferTexto);
}

// [CRÍTICO — restaurado] Confiabilidade da análise, com estado especial de ruptura.
void calcularConfiabilidadeAnalise()
{
  String bufferTexto = "";

  if (analiseRisco.statusTexto == "RUPTURA")
  {
    detalhesConfiab = {0, 0, 0, 0, 0, 0, 0, 100, true};
    analiseRisco.confiabilidade = 100;
    bufferTexto += "[Conf] RUPTURA - confiabilidade forcada para 100%\r\n";
    Serial.print(bufferTexto);
    return;
  }

  detalhesConfiab.estadoEspecialRuptura = false;
  detalhesConfiab.descontoSensor = dadosLocais.sensorOK ? 0 : 40;

  if (!dadosBNDMET.apiDisponivel)
  {
    detalhesConfiab.descontoBndmetFora = 25;
    detalhesConfiab.descontoQualidadeBndmet = 0;
  }
  else
  {
    detalhesConfiab.descontoBndmetFora = 0;
    detalhesConfiab.descontoQualidadeBndmet = (dadosBNDMET.qualidadeDados < 80) ? 10 : 0;
  }

  bool owmDisponivel = (dadosMeteo.timestamp > 0);
  detalhesConfiab.descontoOWM = owmDisponivel ? 0 : 15;
  detalhesConfiab.descontoWifi = wifiConectado ? 0 : 10;
  detalhesConfiab.descontoBuffer = (totalLeiturasSensor < 5) ? 10 : 0;

  detalhesConfiab.totalDesconto =
      detalhesConfiab.descontoSensor + detalhesConfiab.descontoBndmetFora + detalhesConfiab.descontoQualidadeBndmet + detalhesConfiab.descontoOWM + detalhesConfiab.descontoWifi + detalhesConfiab.descontoBuffer;

  detalhesConfiab.resultado = max(0, 100 - detalhesConfiab.totalDesconto);
  analiseRisco.confiabilidade = detalhesConfiab.resultado;

  bufferTexto += "[Conf] Base=100";
  if (detalhesConfiab.descontoSensor)
    bufferTexto += " | -40 sensor";
  if (detalhesConfiab.descontoBndmetFora)
    bufferTexto += " | -25 BNDMET fora";
  if (detalhesConfiab.descontoQualidadeBndmet)
    bufferTexto += " | -10 qualidade BNDMET(" + String(dadosBNDMET.qualidadeDados) + "%)";
  if (detalhesConfiab.descontoOWM)
    bufferTexto += " | -15 OWM fora";
  if (detalhesConfiab.descontoWifi)
    bufferTexto += " | -10 WiFi";
  if (detalhesConfiab.descontoBuffer)
    bufferTexto += " | -10 buffer(" + String(totalLeiturasSensor) + " leit.)";

  bufferTexto += " = " + String(detalhesConfiab.resultado) + "%\r\n";

  Serial.print(bufferTexto);
}

// [CRÍTICO — restaurado] Gera a mensagem de recomendação operacional.
void gerarRecomendacaoDetalhada()
{
  if (simulacaoAtiva)
    return; // preserva o texto "[Simulação]" durante simulação

  String r;
  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA)
  {
    r = "RUPTURA - EVACUACAO IMEDIATA! Umidade acima da linha critica";
  }
  else if (analiseRisco.riscoIntegrado > LIMIAR_AMARELO)
  {
    r = "CRITICO - Evacuar area de risco imediatamente";
  }
  else if (analiseRisco.riscoIntegrado > LIMIAR_VERDE)
  {
    if (previsao.chuvaFutura24h > 5.0f)
      r = "ATENCAO - Chuva prevista, aumentar frequencia de monitoramento";
    else
      r = "ATENCAO - Situacao controlada, manter vigilancia";
  }
  else
  {
    r = "NORMAL - Continuar monitoramento de rotina";
  }
  if (dadosBNDMET.precipitacao7d > 50.0f)
    r += " | Solo saturado por chuvas recentes";
  if (!dadosBNDMET.apiDisponivel)
    r += " | Dados BNDMET indisponiveis";
  if (analiseRisco.amplificado)
    r += " | Amplificacao de risco ativa";
  analiseRisco.recomendacao = r;
}

// [IMPORTANTE — restaurado] Atualiza os arrays de histórico/tendência.
void atualizarHistoricoAnalise()
{
  historicoUmidade[indiceHistorico] = dadosLocais.umidadeSolo;
  historicoPrecipitacao[indiceHistorico] = dadosBNDMET.precipitacao24h;
  historicoRisco[indiceHistorico] = analiseRisco.riscoIntegrado;
  indiceHistorico = (indiceHistorico + 1) % 10;

  String bufferTexto = "";
  bufferTexto += "Historico atualizado - indice: " + String(indiceHistorico) + "\r\n";

  Serial.print(bufferTexto);
}

// ============================================================
//  OVERRIDE DE RUPTURA
// ============================================================
void acionarRuptura()
{
  analiseRisco.riscoIntegrado = 1.0f;
  analiseRisco.indiceRisco = 100;
  analiseRisco.nivelAlerta = "VERMELHO";
  analiseRisco.statusTexto = "RUPTURA";
  analiseRisco.recomendacao = "RUPTURA - EVACUACAO IMEDIATA! Umidade acima da linha critica (" + String(dadosLocais.umidadeSolo, 1) + "% >= " + String(UMIDADE_RUPTURA, 0) + "%)";
  analiseRisco.amplificado = false;
  analiseRisco.vLencol = 0.40f;
  analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;
  statusSistema = 2;
  buzzerAtivo = true;

  String bufferTexto = "";
  bufferTexto += "\r\n🔴 ========================================\r\n";
  bufferTexto += " RUPTURA - UMIDADE ACIMA DO LIMIAR!\r\n";
  bufferTexto += " Umidade: " + String(dadosLocais.umidadeSolo, 2) + "% | Limiar: " + String(UMIDADE_RUPTURA, 2) + "%\r\n";
  bufferTexto += "🔴 ========================================\r\n";
  Serial.print(bufferTexto);

  calcularConfiabilidadeAnalise();

  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, HIGH);
  tone(PIN_BUZZER, 2400);
}

// ============================================================
//  HARDWARE — LEDs e Buzzer
// ============================================================
void controlarSistemaFisico()
{
  String bufferTexto = "";

  if (modoManual)
  {
    bufferTexto += "⚠️ Modo manual - hardware nao alterado\r\n";
    Serial.print(bufferTexto);
    return;
  }
  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA)
    return;

  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  buzzerAtivo = false;

  switch (statusSistema)
  {
  case 0:
    digitalWrite(PIN_LED_VERDE, HIGH);
    bufferTexto += "🟢 LED VERDE - SEGURO | Umidade: " + String(dadosLocais.umidadeSolo, 2) + "% | Risco: " + String(analiseRisco.indiceRisco) + "%\r\n";
    break;
  case 1:
    digitalWrite(PIN_LED_AMARELO, HIGH);
    bufferTexto += "🟡 LED AMARELO - ATENCAO | Umidade: " + String(dadosLocais.umidadeSolo, 2) + "% | Risco: " + String(analiseRisco.indiceRisco) + "%\r\n";
    break;
  case 2:
    digitalWrite(PIN_LED_VERMELHO, HIGH);
    buzzerAtivo = true;
    bufferTexto += "🔴 LED VERMELHO - CRITICO | Umidade: " + String(dadosLocais.umidadeSolo, 2) + "% | Risco: " + String(analiseRisco.indiceRisco) + "%\r\n";
    break;
  }

  Serial.print(bufferTexto);
}

void ativarAlarmeBuzzer()
{
  static unsigned long ultimoBeep = 0;
  static bool estadoBuzzer = false;

  if (!buzzerAtivo)
  {
    noTone(PIN_BUZZER);
    estadoBuzzer = false;
    return;
  }

  unsigned long intervalo = 1000UL;
  if (analiseRisco.indiceRisco >= 90)
    intervalo = 300UL;
  else if (analiseRisco.indiceRisco >= 85)
    intervalo = 500UL;
  else if (analiseRisco.indiceRisco >= 80)
    intervalo = 800UL;

  if (millis() - ultimoBeep >= intervalo)
  {
    estadoBuzzer = !estadoBuzzer;
    digitalWrite(PIN_BUZZER, estadoBuzzer);
    ultimoBeep = millis();
    if (estadoBuzzer)
    {
      unsigned int freq = 2000;
      if (analiseRisco.indiceRisco >= 90)
        freq = 2400;
      else if (analiseRisco.indiceRisco >= 85)
        freq = 2200;
      else if (analiseRisco.indiceRisco >= 80)
        freq = 2000;
      tone(PIN_BUZZER, freq, intervalo / 2);
      ultimoBeep = millis();
    }
  }
}

// ============================================================
//  UTILITÁRIOS DE DATA/HORA
// ============================================================
String obtenerFormatData(time_t t)
{
  struct tm *tmStruct = localtime(&t);
  char buf[11];
  strftime(buf, sizeof(buf), "%Y-%m-%d", tmStruct);
  return String(buf);
}

// [IMPORTANTE — novo] Timestamp ISO-8601 real da leitura, para incluir no payload.
String obterTimestampISO()
{
  time_t agora = time(nullptr);
  if (agora < 1000000000UL)
    return ""; // NTP ainda não sincronizado
  struct tm *tmStruct = gmtime(&agora);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tmStruct);
  return String(buf);
}

// ============================================================
//  BNDMET — I006 (diário) + I175 (horário)
//  [CRÍTICO — corrigido] Estrutura real: doc["data"]["data"] = array de pares
//  [timestamp_ms, valor_ou_null], não valores soltos.
// ============================================================
void buscarDadosDiarios_I006()
{
  if (time(nullptr) < 1000000000UL)
  {
    String bufferTexto = "";
    bufferTexto += "[BNDMET I006] PULADO - horario ainda nao sincronizado.\r\n";
    Serial.print(bufferTexto);
    return;
  }

  // A partir daqui, o NTP já sincronizou — mesmo que a chamada falhe adiante,
  // não é mais uma leitura "pré-NTP" (mesma distinção que o v19 usa em
  // analisarQualidadeDados() pra não contaminar a média de qualidade BNDMET).
  bndmetInicializado = true;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  time_t agora = time(nullptr);
  String dtIni = obtenerFormatData(agora - (30L * 86400L));
  String dtFim = obtenerFormatData(agora - 86400L);
  String dtOntem = dtFim;
  String url = "https://" + String(BNDMET_HOST) + "/v1/estacoes/" + String(BNDMET_ESTACAO) + "/fenomenos/" + String(BNDMET_COD_I006) + "?dataInicio=" + dtIni + "&dataFinal=" + dtFim;

  Serial.print("[BNDMET I006] GET ");
  Serial.println(url);
  http.begin(client, url);
  http.addHeader("x-api-key", BNDMET_API_KEY);
  http.setTimeout(30000);
  int code = http.GET();

  String bufferTexto = "";
  bufferTexto += "[BNDMET I006] HTTP code: " + String(code) + "\r\n";

  if (code != 200)
  {
    dadosBNDMET.apiDisponivel = false;
    dadosBNDMET.statusAPI = "FALHA";
    dadosBNDMET.qualidadeDados = 0;
    bufferTexto += "[BNDMET I006] FALHA na consulta.\r\n";
    Serial.print(bufferTexto);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    bufferTexto += "[BNDMET I006] Erro ao parsear JSON: " + String(err.c_str()) + "\r\n";
    Serial.print(bufferTexto);
    dadosBNDMET.apiDisponivel = false;
    return;
  }

  if (doc["data"]["data"].isNull() || !doc["data"]["data"].is<JsonArray>())
  {
    bufferTexto += "[BNDMET I006] Estrutura inesperada - campo data.data ausente\r\n";
    Serial.print(bufferTexto);
    dadosBNDMET.apiDisponivel = false;
    return;
  }

  JsonArray dataArr = doc["data"]["data"].as<JsonArray>();
  int total = dataArr.size();
  int medicoesValidas = 0;
  float soma7d = 0.0f, soma30d = 0.0f, val24h = 0.0f;
  bool achou24h = false;

  // Cada elemento é [timestamp_ms, valor_ou_null] — não um float solto.
  for (int i = 0; i < total; i++)
  {
    JsonArray medicao = dataArr[i];
    if (medicao.size() < 2)
      continue;

    long long tsMed_ms = medicao[0].as<long long>();
    if (tsMed_ms <= 0 || medicao[1].isNull())
      continue;

    time_t tSec = (time_t)(tsMed_ms / 1000LL);
    String dtMed = obtenerFormatData(tSec);
    float chuva = medicao[1].as<float>();
    medicoesValidas++;

    if (dtMed == dtOntem)
    {
      val24h = chuva;
      achou24h = true;
    }

    int diasDoFim = total - 1 - i;
    if (diasDoFim < 7)
      soma7d += chuva;
    soma30d += chuva;
  }

  dadosBNDMET.precipitacao24h = achou24h ? val24h : 0.0f;
  dadosBNDMET.precipitacao7d = soma7d;
  dadosBNDMET.precipitacao30d = soma30d;
  dadosBNDMET.apiDisponivel = true;
  dadosBNDMET.statusAPI = "OK";
  dadosBNDMET.estacao = BNDMET_ESTACAO;
  dadosBNDMET.timestamp = millis();
  dadosBNDMET.qualidadeDados = total > 0 ? (medicoesValidas * 100) / total : 0;

  bufferTexto += "[BNDMET I006] OK - 24h=" + String(dadosBNDMET.precipitacao24h, 2) + " | 7d=" + String(dadosBNDMET.precipitacao7d, 2) + " | 30d=" + String(dadosBNDMET.precipitacao30d, 2) + " mm | Qualidade=" + String(dadosBNDMET.qualidadeDados) + "%\r\n";
  Serial.print(bufferTexto);
}

void buscarDadosHorarios_I175()
{
  if (time(nullptr) < 1000000000UL)
  {
    String bufferTexto = "";
    bufferTexto += "[BNDMET I175] PULADO - horario ainda nao sincronizado.\r\n";
    Serial.print(bufferTexto);
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  time_t agora = time(nullptr);
  String dtHoje = obtenerFormatData(agora);
  struct tm *tmAgora = localtime(&agora);
  int horaAtualMin = tmAgora->tm_hour * 60 + tmAgora->tm_min;

  String url = "https://" + String(BNDMET_HOST) + "/v1/estacoes/" + String(BNDMET_ESTACAO) + "/fenomenos/" + String(BNDMET_COD_I175) + "?dataInicio=" + dtHoje + "&dataFinal=" + dtHoje;

  Serial.print("[BNDMET I175] GET ");
  Serial.println(url);
  http.begin(client, url);
  http.addHeader("x-api-key", BNDMET_API_KEY);
  http.setTimeout(30000);
  int code = http.GET();

  String bufferTexto = "";
  bufferTexto += "[BNDMET I175] HTTP code: " + String(code) + "\r\n";

  if (code != 200)
  {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    bufferTexto += "[BNDMET I175] FALHA na consulta.\r\n";
    Serial.print(bufferTexto);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err || doc["data"]["data"].isNull() || !doc["data"]["data"].is<JsonArray>())
  {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    bufferTexto += "[BNDMET I175] Erro ao parsear ou estrutura inesperada\r\n";
    Serial.print(bufferTexto);
    return;
  }

  JsonArray dataArr = doc["data"]["data"].as<JsonArray>();
  int menorDiff = 99999;
  float melhorVal = 0.0f;

  // Seleciona a medição [timestamp_ms, valor] mais próxima do horário atual.
  for (JsonArray medicao : dataArr)
  {
    if (medicao.size() < 2 || medicao[1].isNull())
      continue;

    long long tsMed_ms = medicao[0].as<long long>();
    if (tsMed_ms <= 0)
      continue;
    time_t tSec = (time_t)(tsMed_ms / 1000LL);
    struct tm *tmMed = localtime(&tSec);
    int medMin = tmMed->tm_hour * 60 + tmMed->tm_min;
    int diff = abs(horaAtualMin - medMin);

    if (diff < menorDiff)
    {
      menorDiff = diff;
      melhorVal = medicao[1].as<float>();
    }
  }

  dadosBNDMET.precipitacaoAtual = melhorVal;
  bufferTexto += "[BNDMET I175] OK - (diff=" + String(menorDiff) + "min): " + String(dadosBNDMET.precipitacaoAtual, 2) + "mm\r\n";
  Serial.print(bufferTexto);
}

// ============================================================
//  OPENWEATHERMAP — /weather + /forecast
// ============================================================
bool buscarWeatherAtual()
{
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = "https://" + String(OWM_HOST) + "/data/2.5/weather?lat=" + String(OWM_LAT) + "&lon=" + String(OWM_LON) + "&appid=" + String(OWM_API_KEY) + "&lang=pt_br&units=metric";
  Serial.print("[OWM Weather] GET ");
  Serial.println(url);
  http.begin(client, url);
  http.setTimeout(15000);
  int code = http.GET();

  String bufferTexto = "";
  bufferTexto += "[OWM Weather] HTTP code: " + String(code) + "\r\n";

  if (code != 200)
  {
    bufferTexto += "[OWM Weather] FALHA na consulta.\r\n";
    Serial.print(bufferTexto);
    http.end();
    return false;
  }

  String payload = http.getString();
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    bufferTexto += "[OWM Weather] Erro ao parsear JSON: " + String(err.c_str()) + "\r\n";
    Serial.print(bufferTexto);
    http.end();
    return false;
  }

  dadosMeteo.temperatura = doc["main"]["temp"] | 0.0f;
  dadosMeteo.umidadeExterna = doc["main"]["humidity"] | 0.0f;
  dadosMeteo.velocidadeVento = doc["wind"]["speed"] | 0.0f;
  if (!doc["weather"][0]["description"].isNull())
    dadosMeteo.descricaoTempo = doc["weather"][0]["description"].as<String>();

  // [IMPORTANTE — corrigido] grnd_level preferido, fallback pressure — e agora
  // REGISTRADO no histórico circular (antes, historicoPressao nunca era escrito).
  float pressaoNova = 0.0f;
  JsonVariant grnd = doc["main"]["grnd_level"];
  if (!grnd.isNull())
  {
    pressaoNova = grnd.as<float>();
  }
  else
  {
    pressaoNova = doc["main"]["pressure"] | 0.0f;
  }

  if (pressaoNova >= PRESSAO_MIN && pressaoNova <= PRESSAO_MAX)
  {
    historicoPressao[indexPressao % HIST_PRESSAO] = pressaoNova;
    indexPressao++;
    if (totalLeiturasPressao < HIST_PRESSAO)
      totalLeiturasPressao++;
    dadosMeteo.pressaoAtmosferica = pressaoNova;
    bufferTexto += "[OWM Weather] Pressao registrada: " + String(pressaoNova, 2) + " hPa | historico: " + String(totalLeiturasPressao) + "/" + String(HIST_PRESSAO) + "\r\n";
  }
  else
  {
    bufferTexto += "[OWM Weather] Pressao fora do range (" + String(pressaoNova, 2) + " hPa) - ignorada\r\n";
  }

  if (!doc["rain"].isNull() && !doc["rain"]["1h"].isNull())
  {
    dadosMeteo.chuvaAtualOWM = doc["rain"]["1h"].as<float>();
  }
  else
  {
    dadosMeteo.chuvaAtualOWM = 0.0f;
  }

  dadosMeteo.timestamp = millis();
  bufferTexto += "[OWM Weather] OK - Temp: " + String(dadosMeteo.temperatura, 1) + "C | Umidade: " + String(dadosMeteo.umidadeExterna, 0) + "% | Pressao: " + String(dadosMeteo.pressaoAtmosferica, 1) + "\r\n";
  Serial.print(bufferTexto);
  http.end();
  return true;
}

bool buscarForecast()
{
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = "https://" + String(OWM_HOST) + "/data/2.5/forecast?lat=" + String(OWM_LAT) + "&lon=" + String(OWM_LON) + "&appid=" + String(OWM_API_KEY) + "&cnt=8&units=metric";
  Serial.print("[OWM Forecast] GET ");
  Serial.println(url);
  http.begin(client, url);
  http.setTimeout(15000);
  int code = http.GET();

  String bufferTexto = "";
  bufferTexto += "[OWM Forecast] HTTP code: " + String(code) + "\r\n";

  if (code != 200)
  {
    bufferTexto += "[OWM Forecast] FALHA na consulta.\r\n";
    Serial.print(bufferTexto);
    http.end();
    return false;
  }

  String payload = http.getString();
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    bufferTexto += "[OWM Forecast] Erro ao parsear JSON: " + String(err.c_str()) + "\r\n";
    Serial.print(bufferTexto);
    http.end();
    return false;
  }

  float soma = 0;
  JsonArray list = doc["list"].as<JsonArray>();
  for (JsonObject item : list)
  {
    float chuva3h = item["rain"]["3h"] | 0.0f;
    float pop = item["pop"] | 0.0f;
    soma += (chuva3h > 0.0f) ? chuva3h : (pop * 3.0f);
  }
  previsao.chuvaFutura24h = soma;
  previsao.intensidadePrevisao = obterIntensidadePrevisao();
  previsao.fatorIntensidade = calcularFatorPrevisaoIntensidade(previsao.intensidadePrevisao);
  previsao.timestamp = millis();

  bufferTexto += "[OWM Forecast] OK - Chuva prevista 24h: " + String(soma, 1) + "mm (" + previsao.intensidadePrevisao + ", fator=" + String(previsao.fatorIntensidade, 2) + ")\r\n";
  Serial.print(bufferTexto);
  http.end();
  return true;
}

// ============================================================
//  WIFI E CONECTIVIDADE
// ============================================================
void conectarSistemas()
{
  String bufferTexto = "";
  bufferTexto += "🌐 Conectando ao WiFi...\r\n";
  Serial.print(bufferTexto);
  bufferTexto = ""; // Limpa para o próximo bloco

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20)
  {
    delay(500);
    Serial.print("."); // Feedback visual imediato mantido na mesma linha
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConectado = true;
    bufferTexto += "\r\n[OK] WiFi Conectado! IP: " + WiFi.localIP().toString() + "\r\n";
  }
  else
  {
    wifiConectado = false;
    bufferTexto += "\r\n[ERR] Falha na conexao WiFi\r\n";
  }

  Serial.print(bufferTexto);
}

// [IMPORTANTE — restaurado] Reconecta o WiFi se necessário, sem bloquear o loop por muito tempo.
void garantirWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    String bufferTexto = "";
    bufferTexto += "[WiFi] Reconectando...\r\n";
    Serial.print(bufferTexto);

    WiFi.reconnect();
    delay(2000);
    wifiConectado = (WiFi.status() == WL_CONNECTED);
  }
}

// [IMPORTANTE — restaurado] Watchdog leve de conectividade, chamado periodicamente do loop().
void verificarConectividade()
{
  wifiConectado = (WiFi.status() == WL_CONNECTED);
  if (!wifiConectado)
  {
    String bufferTexto = "";
    bufferTexto += "WiFi desconectado - tentando reconectar...\r\n";
    Serial.print(bufferTexto);

    WiFi.reconnect();
    delay(3000);
    wifiConectado = (WiFi.status() == WL_CONNECTED);
    if (wifiConectado)
      tentativasReconexao = 0;
    else
      tentativasReconexao++;
  }
  if (wifiConectado && !apiConectada)
    verificarStatusAPI();
}

bool verificarStatusAPI()
{
  if (!wifiConectado)
    return false;

  HTTPClient http;
  http.begin(String(API_BASE_URL) + String(API_STATUS_ENDPOINT));
  http.setTimeout(5000);
  int code = http.GET();

  if (code == 200)
  {
    apiConectada = true;
    http.end();
    return true;
  }

  apiConectada = false;

  // Aplicacao do padrao de Buffer de Texto Unificado
  String bufferTexto = "";
  bufferTexto += "[API] Nao responde: HTTP " + String(code) + "\r\n";
  Serial.print(bufferTexto);

  http.end();
  return false;
}

// ============================================================
//  ENVIO PARA API PRÓPRIA
//  [CRÍTICO — expandido] Payload completo, igual ao criarPayloadJSON() do v19.
// ============================================================
String criarPayloadJSON()
{
  JsonDocument doc;

  doc["timestamp"] = obterTimestampISO();
  doc["umidadeSolo"] = dadosLocais.umidadeSolo;
  doc["valorAdc"] = dadosLocais.valorADC;
  doc["sensorOk"] = dadosLocais.sensorOK;
  doc["fatorLocal"] = dadosLocais.fatorLocal;

  doc["estacao"] = dadosBNDMET.estacao;
  doc["precipitacao24h"] = dadosBNDMET.precipitacao24h;
  doc["precipitacao7d"] = dadosBNDMET.precipitacao7d;
  doc["precipitacao30d"] = dadosBNDMET.precipitacao30d;
  doc["precipitacaoAtual"] = dadosBNDMET.precipitacaoAtual;
  doc["statusApiBndmet"] = dadosBNDMET.statusAPI;
  doc["qualidadeDadosBndmet"] = dadosBNDMET.qualidadeDados;

  doc["statusApiOwm"] = (dadosMeteo.timestamp > 0) ? "OK" : "FALHA";
  doc["temperatura"] = dadosMeteo.temperatura;
  doc["umidadeExterna"] = dadosMeteo.umidadeExterna;
  doc["pressaoAtmosferica"] = dadosMeteo.pressaoAtmosferica;
  doc["velocidadeVento"] = dadosMeteo.velocidadeVento;
  doc["chuvaAtualOWM"] = dadosMeteo.chuvaAtualOWM;
  doc["descricaoTempo"] = dadosMeteo.descricaoTempo;

  doc["chuvaFutura24h"] = previsao.chuvaFutura24h;
  doc["intensidadePrevisao"] = previsao.intensidadePrevisao;
  doc["fatorIntensidade"] = previsao.fatorIntensidade;

  doc["riscoIntegrado"] = analiseRisco.riscoIntegrado;
  doc["indiceRisco"] = analiseRisco.indiceRisco;
  doc["nivelAlerta"] = analiseRisco.nivelAlerta;
  doc["recomendacao"] = analiseRisco.recomendacao;
  doc["confiabilidade"] = analiseRisco.confiabilidade;
  doc["amplificado"] = analiseRisco.amplificado;
  doc["taxaVariacaoUmidade"] = calcularTaxaVariacao();

  doc["vLencol"] = analiseRisco.vLencol;
  doc["vChuvaAtual"] = analiseRisco.vChuvaAtual;
  doc["vChuvaHistorica"] = analiseRisco.vChuvaHistorica;
  doc["vChuvaMensal"] = analiseRisco.vChuvaMensal;
  doc["vChuvaFutura"] = analiseRisco.vChuvaFutura;
  doc["vTaxaVariacao"] = analiseRisco.vTaxaVariacao;
  doc["vPressao"] = analiseRisco.vPressao;

  doc["statusSistema"] = statusSistema;
  doc["buzzerAtivo"] = buzzerAtivo;
  doc["modoManual"] = modoManual;
  doc["wifiConectado"] = wifiConectado;

  JsonObject dadosBrutos = doc["dadosBrutos"].to<JsonObject>();
  dadosBrutos["uptime"] = millis() / 1000;
  dadosBrutos["freeHeap"] = ESP.getFreeHeap();
  dadosBrutos["rssi"] = WiFi.RSSI();
  dadosBrutos["tentativasEnvio"] = tentativasEnvioAPI;
  dadosBrutos["tentativasReconexao"] = tentativasReconexao;
  // [TESTE] Rastreabilidade: sinaliza no banco que este ciclo usou chuva
  // sintética (comando "chuva sintetica"), para nunca confundir com leitura
  // real de BNDMET/OWM ao analisar o dataset depois.
  dadosBrutos["chuva_sintetica_ativa"] = chuvaSinteticaAtiva;
  dadosBrutos["bndmet_inicializado"] = bndmetInicializado;
  dadosBrutos["aguardando_reset_ruptura"] = aguardandoResetRuptura;

  JsonObject confiabDetalhes = dadosBrutos["confiabilidade_detalhes"].to<JsonObject>();
  confiabDetalhes["base"] = 100;
  confiabDetalhes["resultado"] = detalhesConfiab.resultado;
  confiabDetalhes["total_desconto"] = detalhesConfiab.totalDesconto;
  confiabDetalhes["estado_especial"] = detalhesConfiab.estadoEspecialRuptura ? "RUPTURA" : (const char *)nullptr;
  JsonObject descontos = confiabDetalhes["descontos"].to<JsonObject>();
  descontos["sensor_falha"] = detalhesConfiab.descontoSensor;
  descontos["bndmet_indisponivel"] = detalhesConfiab.descontoBndmetFora;
  descontos["qualidade_bndmet"] = detalhesConfiab.descontoQualidadeBndmet;
  descontos["owm_indisponivel"] = detalhesConfiab.descontoOWM;
  descontos["wifi_desconectado"] = detalhesConfiab.descontoWifi;
  descontos["buffer_insuficiente"] = detalhesConfiab.descontoBuffer;

  String saida;
  serializeJson(doc, saida);
  return saida;
}

bool enviarDadosParaAPI()
{
  String bufferTexto = "";

  if (!wifiConectado)
  {
    bufferTexto += "[API] WiFi desconectado - nao e possivel enviar dados\r\n";
    Serial.print(bufferTexto);
    return false;
  }

  bufferTexto += "📤 Enviando dados para API...\r\n";
  Serial.print(bufferTexto);
  bufferTexto = ""; // Limpa o buffer para as proximas mensagens

  String payload = criarPayloadJSON();

  HTTPClient http;
  String url = String(API_BASE_URL) + String(API_ENDPOINT);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  if (strlen(API_TOKEN) > 0)
    http.addHeader("Authorization", "Bearer " + String(API_TOKEN));
  http.setTimeout(15000);

  int code = http.POST(payload);

  if (code == 200 || code == 201)
  {
    bufferTexto += "[API] Dados enviados com sucesso\r\n";
    Serial.print(bufferTexto);
    apiConectada = true;
    tentativasEnvioAPI = 0;
    http.end();
    return true;
  }

  bufferTexto += "[API] Erro ao enviar - HTTP " + String(code) + "\r\n";
  if (code > 0)
  {
    bufferTexto += "[API] " + http.getString() + "\r\n";
  }
  Serial.print(bufferTexto);

  apiConectada = false;
  tentativasEnvioAPI++;
  http.end();
  return false;
}

// ============================================================
//  STATUS / DEBUG / COMANDOS
// ============================================================
void mostrarStatusConectividade()
{
  String bufferTexto = "";
  bufferTexto += "WiFi: " + String(wifiConectado ? "OK" : "ERR") + " | BNDMET: " + dadosBNDMET.statusAPI + " | Precip24h: " + String(dadosBNDMET.precipitacao24h, 1) + "mm | Precip7d: " + String(dadosBNDMET.precipitacao7d, 1) + "mm\r\n";
  bufferTexto += "OWM - Temp: " + String(dadosMeteo.temperatura, 1) + "C | Umidade: " + String(dadosMeteo.umidadeExterna, 0) + "% | Pressao: " + String(dadosMeteo.pressaoAtmosferica, 1) + " | Chuva prevista 24h: " + String(previsao.chuvaFutura24h, 1) + "mm\r\n";
  bufferTexto += "Recomendacao: " + analiseRisco.recomendacao + " | Confiabilidade: " + String(analiseRisco.confiabilidade) + "%\r\n";
  Serial.print(bufferTexto);
}

void mostrarAnaliseDetalhada()
{
  String bufferTexto = "";
  bufferTexto += "\r\n===== ANALISE COMPLETA DE RISCO - TCC =====\r\n";
  bufferTexto += "DADOS LOCAIS: Umidade=" + String(dadosLocais.umidadeSolo, 2) + "% | ADC=" + String(dadosLocais.valorADC) + " | Fator=" + String(dadosLocais.fatorLocal, 3) + "\r\n";
  bufferTexto += "BNDMET: Estacao=" + String(BNDMET_ESTACAO) + " | Status=" + dadosBNDMET.statusAPI + " | 24h=" + String(dadosBNDMET.precipitacao24h, 2) + "mm | 7d=" + String(dadosBNDMET.precipitacao7d, 2) + "mm | 30d=" + String(dadosBNDMET.precipitacao30d, 2) + "mm | Qualidade=" + String(dadosBNDMET.qualidadeDados) + "%\r\n";
  bufferTexto += "OWM: Pressao=" + String(dadosMeteo.pressaoAtmosferica, 2) + "hPa | Temp=" + String(dadosMeteo.temperatura, 2) + "C | ChuvaAtual=" + String(dadosMeteo.chuvaAtualOWM, 2) + "mm/h | Forecast24h=" + String(previsao.chuvaFutura24h, 2) + "mm (" + previsao.intensidadePrevisao + ", fator=" + String(previsao.fatorIntensidade, 2) + ")\r\n";
  bufferTexto += "--- Equacao 5 TCC - componentes ---\r\n";
  bufferTexto += "  V_lencol=" + String(analiseRisco.vLencol, 3) + "  V_ch.atual=" + String(analiseRisco.vChuvaAtual, 3) + "  V_ch.hist=" + String(analiseRisco.vChuvaHistorica, 3) + "  V_ch.mensal=" + String(analiseRisco.vChuvaMensal, 3) + "\r\n";
  bufferTexto += "  V_ch.futura=" + String(analiseRisco.vChuvaFutura, 3) + "  V_taxa_var=" + String(analiseRisco.vTaxaVariacao, 3) + "  V_pressao=" + String(analiseRisco.vPressao, 3) + "  Amplificado=" + String(analiseRisco.amplificado ? "SIM (1,2x)" : "NAO") + "\r\n";
  bufferTexto += "FATOR RISCO=" + String(analiseRisco.riscoIntegrado, 3) + " | INDICE=" + String(analiseRisco.indiceRisco) + "% | STATUS=" + analiseRisco.statusTexto + " | Confiabilidade=" + String(analiseRisco.confiabilidade) + "%\r\n";
  bufferTexto += "Recomendacao: " + analiseRisco.recomendacao + "\r\n";
  bufferTexto += "============================================\r\n";
  Serial.print(bufferTexto);
}

void debugConectividade()
{
  String bufferTexto = "";
  bufferTexto += "\r\n===== DEBUG CONECTIVIDADE =====\r\n";
  bufferTexto += "Status WiFi: " + String(WiFi.status()) + "\r\n";
  if (wifiConectado)
  {
    bufferTexto += "  SSID: " + WiFi.SSID() + "\r\n";
    bufferTexto += "  IP: " + WiFi.localIP().toString() + "\r\n";
    bufferTexto += "  RSSI: " + String(WiFi.RSSI()) + " dBm\r\n";
  }
  WiFiClient tc;
  bool internetOk = tc.connect("8.8.8.8", 53);
  bufferTexto += "  Internet (8.8.8.8:53): " + String(internetOk ? "OK" : "FALHA") + "\r\n";
  tc.stop();
  bufferTexto += "================================\r\n";
  Serial.print(bufferTexto);
}

// [IMPORTANTE — restaurado, adaptado] No Wokwi o potenciômetro é linear e
// não sofre deriva como o higrômetro capacitivo real — não há persistência
// em EEPROM entre simulações (cada run do Wokwi começa do zero), mas o
// comando permite recalibrar em tempo de execução se necessário.
void recalibrarSensor()
{
  int leitura = analogRead(PIN_HIGROMETRO);

  // Aplicacao do padrao de Buffer de Texto Unificado
  String bufferTexto = "";
  bufferTexto += "\r\n[Calibracao] Lendo ADC atual como referencia...\r\n";
  bufferTexto += "Leitura atual: " + String(leitura) + " (range esperado do potenciometro Wokwi: 0-4095)\r\n";
  bufferTexto += "Potenciometro do Wokwi ja e linear (SENSOR_SECO=0, SENSOR_UMIDO=4095).\r\n";
  bufferTexto += "Recalibracao manual nao e necessaria neste simulador.\r\n";

  Serial.print(bufferTexto);
}

void testarCalculosTCC()
{
  String bufferTexto = "";
  bufferTexto += "[Teste] Testando calculos com dados reais capturados...\r\n";
  float u = dadosLocais.umidadeSolo;
  float p24 = dadosBNDMET.precipitacao24h;
  float p7 = dadosBNDMET.precipitacao7d;
  float p30 = dadosBNDMET.precipitacao30d;
  float fc = previsao.chuvaFutura24h;

  float vL = calcularVLencolFreatico(u);
  float vCA = calcularVChuvaAtual(p24);
  float vCH = calcularVChuvaHistorica(p7);
  float vCM = calcularVChuvaMensal(p30);
  float vCF = calcularVChuvaFutura(fc);
  float total = vL + vCA + vCH + vCM + vCF;

  float fatorLen = calcularFatorLencolFreatico(u);
  bool amplif = (fatorLen >= LIMIAR_SOLO_SAT) && (fc >= 5.0f);
  if (amplif)
    total *= FATOR_AMPLIF;

  const char *nivel = (total <= LIMIAR_VERDE) ? "VERDE" : (total <= LIMIAR_AMARELO) ? "AMARELO"
                                                                                    : "VERMELHO";

  bufferTexto += "  Umidade=" + String(u, 2) + "% P24=" + String(p24, 2) + "mm P7=" + String(p7, 2) + "mm P30=" + String(p30, 2) + "mm FC=" + String(fc, 2) + "mm\r\n";
  bufferTexto += "  V_lencol=" + String(vL, 3) + " V_ch.atual=" + String(vCA, 3) + " V_ch.hist=" + String(vCH, 3) + " V_ch.mensal=" + String(vCM, 3) + " V_ch.futura=" + String(vCF, 3) + " TOTAL=" + String(total, 3) + "\r\n";
  if (amplif)
    bufferTexto += "  Amplificacao 1,2x aplicada\r\n";
  bufferTexto += "  Resultado: LED " + String(nivel) + "\r\n";
  Serial.print(bufferTexto);
}

void executarTesteCompleto()
{
  String bufferTexto = "";
  bufferTexto += "\r\n===== TESTE COMPLETO DO SISTEMA =====\r\n";
  bufferTexto += "Teste 1: Hardware\r\n";
  Serial.print(bufferTexto);
  for (int p : {(int)PIN_LED_VERDE, (int)PIN_LED_AMARELO, (int)PIN_LED_VERMELHO})
  {
    digitalWrite(p, HIGH);
    delay(300);
    digitalWrite(p, LOW);
  }
  digitalWrite(PIN_BUZZER, HIGH);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);

  bufferTexto = "";
  bufferTexto += "  LEDs e buzzer OK\r\n";
  bufferTexto += "Teste 2: Conectividade\r\n";
  bufferTexto += "  WiFi: " + String(wifiConectado ? "OK" : "FALHA") + "\r\n";
  bufferTexto += "  BNDMET: " + String(dadosBNDMET.apiDisponivel ? "OK" : "FALHA") + "\r\n";
  Serial.print(bufferTexto);
  debugConectividade();

  dadosLocais.valorADC = analogRead(PIN_HIGROMETRO);
  dadosLocais.umidadeSolo = constrain(((float)dadosLocais.valorADC / 4095.0f) * 100.0f, 0.0f, 100.0f);

  bufferTexto = "";
  bufferTexto += "Teste 3: Sensor de umidade\r\n";
  bufferTexto += "  Leitura: " + String(dadosLocais.umidadeSolo, 2) + "%\r\n";
  Serial.print(bufferTexto);

  bufferTexto = "";
  bufferTexto += "Teste 4: APIs BNDMET\r\n";

  if (wifiConectado)
  {
    Serial.print(bufferTexto);
    buscarDadosDiarios_I006();
    buscarDadosHorarios_I175();
  }
  else
  {
    bufferTexto += "WiFi desconectado\r\n";
    Serial.print(bufferTexto);
  }

  bufferTexto = "";
  bufferTexto += "Teste 5: OpenWeatherMap\r\n";

  if (wifiConectado)
  {
    Serial.print(bufferTexto);
    buscarWeatherAtual();
    buscarForecast();
  }
  else
  {
    bufferTexto += "WiFi desconectado\r\n";
    Serial.print(bufferTexto);
  }

  bufferTexto = "";
  bufferTexto += "Teste 6: Calculos TCC\r\n";
  Serial.print(bufferTexto);

  testarCalculosTCC();

  bufferTexto = "";
  bufferTexto += "Teste 7: Analise de risco\r\n";
  Serial.print(bufferTexto);

  analisarRiscoIntegrado();

  bufferTexto = "";
  bufferTexto += "\r\n===== TESTE COMPLETO FINALIZADO =====\r\n";
  Serial.print(bufferTexto);
}

void resetarSistema()
{
  String bufferTexto = "";
  bufferTexto += "\r\n[Reset] Resetando sistema...\r\n";
  bufferTexto += "[Reset] Reiniciando em 3s...\r\n";
  Serial.print(bufferTexto);
  Serial.flush(); // Garante a vazao total do texto antes do restart do hardware

  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  statusSistema = 0;
  buzzerAtivo = false;
  modoManual = false;
  simulacaoAtiva = false;
  for (int i = 0; i < 10; i++)
  {
    historicoUmidade[i] = 0;
    historicoPrecipitacao[i] = 0;
    historicoRisco[i] = 0;
  }
  indiceHistorico = 0;

  delay(3000);
  ESP.restart();
}

void mostrarComandosDisponiveis()
{
  String bufferTexto = "";
  bufferTexto += "\r\n===== COMANDOS DISPONIVEIS =====\r\n";
  bufferTexto += "  status   - Status do sistema\r\n";
  bufferTexto += "  analise  - Executar analise de risco\r\n";
  bufferTexto += "  debug    - Analise detalhada (Equacoes TCC)\r\n";
  bufferTexto += "  calibrar - Verificar calibracao do sensor\r\n";
  bufferTexto += "  reset    - Resetar sistema\r\n";
  bufferTexto += "  teste    - Teste completo de todos os modulos\r\n";
  bufferTexto += "  enviar   - Forcar envio para API\r\n";
  bufferTexto += "  api      - Testar conexao com API\r\n";
  bufferTexto += "  help     - Este menu\r\n";
  bufferTexto += "--- Simulacao de nivel (demonstracao) ---\r\n";
  bufferTexto += "  alerta verde    - Forca nivel VERDE\r\n";
  bufferTexto += "  alerta amarelo  - Forca nivel AMARELO\r\n";
  bufferTexto += "  alerta vermelho - Forca nivel VERMELHO\r\n";
  bufferTexto += "  (proxima leitura real do sensor restaura o nivel)\r\n";
  bufferTexto += "--- Chuva sintetica (escalada natural pela equacao) ---\r\n";
  bufferTexto += "  chuva sintetica - Fixa precip/previsao no teto (V_len/V_taxa/V_pressao reais)\r\n";
  bufferTexto += "  chuva real      - Desativa, retomando BNDMET/OWM reais\r\n";
  bufferTexto += "  (com chuva sintetica ativa: umidade ~2-15% -> AMARELO | >=17,5% -> VERMELHO)\r\n";
  bufferTexto += "===================================\r\n";
  Serial.print(bufferTexto);
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);
  String bufferTexto = "";
  bufferTexto += "\r\n==============================================\r\n";
  bufferTexto += " SIMULADOR DE BARRAGEM - DADOS REAIS ONLINE\r\n";
  bufferTexto += "==============================================\r\n";
  Serial.print(bufferTexto);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARELO, OUTPUT);
  pinMode(PIN_LED_VERMELHO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_HIGROMETRO, INPUT);

  for (int p : {(int)PIN_LED_VERDE, (int)PIN_LED_AMARELO, (int)PIN_LED_VERMELHO})
  {
    digitalWrite(p, HIGH);
    delay(300);
    digitalWrite(p, LOW);
  }

  conectarSistemas();

  configTime(-3 * 3600, 0, "pool.ntp.org");

  // Espera ativa (com timeout) até o NTP sincronizar. Sem isso, as chamadas
  // ao BNDMET saem de imediato (time(nullptr) ainda não passou de 1000000000UL).
  if (wifiConectado)
  {
    String bufferTexto = "";
    bufferTexto += "Sincronizando NTP";
    Serial.print(bufferTexto);
    Serial.flush();

    int tentativasNTP = 0;
    while (time(nullptr) < 1000000000UL && tentativasNTP < 20)
    {
      delay(500);
      Serial.print("."); // Feedback visual mantido na mesma linha para o usuario
      Serial.flush();
      tentativasNTP++;
    }

    bufferTexto = ""; // Limpa para a mensagem final do bloco
    if (time(nullptr) >= 1000000000UL)
    {
      bufferTexto += "\r\n[OK] Horario sincronizado.\r\n";
    }
    else
    {
      bufferTexto += "\r\n[ERR] NTP nao sincronizou a tempo.\r\n";
    }
    Serial.print(bufferTexto);
  }

  dadosLocais.valorADC = analogRead(PIN_HIGROMETRO);
  dadosLocais.umidadeSolo = constrain(((float)dadosLocais.valorADC / 4095.0f) * 100.0f, 0.0f, 100.0f);
  dadosLocais.fatorLocal = calcularFatorLencolFreatico(dadosLocais.umidadeSolo);
  // [CRÍTICO — corrigido] sensorOK real, calculado pelos limites do ADC (0-4095 no
  // ESP32/Wokwi), em vez de fixo em true. Extremos indicam curto-circuito (0) ou
  // sonda/potenciômetro desconectado (4095), igual à lógica do tcc_versao_19.ino.
  dadosLocais.sensorOK = (dadosLocais.valorADC > 0 && dadosLocais.valorADC < 4095);
  totalLeiturasSensor++;
  bufferUmidade[bufferIndex] = dadosLocais.umidadeSolo;
  bufferIndex = (bufferIndex + 1) % BUFFER_UMIDADE;

  if (wifiConectado)
  {
    String bufferTexto = "";
    bufferTexto += "--- Iniciando consultas as APIs externas ---\r\n";
    Serial.print(bufferTexto);

    buscarDadosDiarios_I006();
    buscarDadosHorarios_I175();
    buscarWeatherAtual();
    buscarForecast();

    bufferTexto = "";
    bufferTexto += "--- Consultas concluidas ---\r\n";
    Serial.print(bufferTexto);
  }

  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA)
  {
    acionarRuptura();
  }
  else
  {
    analisarRiscoIntegrado();
  }
  if (wifiConectado)
  {
    String bufferTexto = "";
    bufferTexto += "📤 Enviando dados iniciais...\r\n";
    Serial.print(bufferTexto);

    enviarDadosParaAPI();
  }

  unsigned long agr = millis();
  ultimaLeituraSensor = agr;
  ultimaLeituraBNDMET = agr;
  ultimaOWM = agr;
  ultimaAnalise = agr;
  ultimoEnvioAPI = agr;

  mostrarComandosDisponiveis();
  mostrarStatusConectividade();
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop()
{
  unsigned long agora = millis();
  static uint8_t contagemRetornoRuptura = 0;

  // Leitura do sensor
  if (agora - ultimaLeituraSensor >= obterIntervaloSensor())
  {
    if (!modoManual)
    {
      dadosLocais.valorADC = analogRead(PIN_HIGROMETRO);
      dadosLocais.umidadeSolo = constrain(((float)dadosLocais.valorADC / 4095.0f) * 100.0f, 0.0f, 100.0f);
      dadosLocais.fatorLocal = calcularFatorLencolFreatico(dadosLocais.umidadeSolo);
      // [CRÍTICO — corrigido] mesma lógica de sensorOK real aplicada no loop().
      dadosLocais.sensorOK = (dadosLocais.valorADC > 0 && dadosLocais.valorADC < 4095);
      totalLeiturasSensor++;
      bufferUmidade[bufferIndex] = dadosLocais.umidadeSolo;
      bufferIndex = (bufferIndex + 1) % BUFFER_UMIDADE;
      if (bufferIndex == 0)
        bufferCheio = true;

      String bufferTexto = "";
      bufferTexto += "Sensor - ADC: " + String(dadosLocais.valorADC) + " | Umidade: " + String(dadosLocais.umidadeSolo, 2) + "%\r\n";
      Serial.print(bufferTexto);
    }

    // [IMPORTANTE — restaurado] Cooldown de 3 leituras seguras antes de sair da ruptura.
    if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA && !simulacaoAtiva)
    {
      contagemRetornoRuptura = 0;
      aguardandoResetRuptura = false;
      acionarRuptura();
    }
    else if (analiseRisco.nivelAlerta == "VERMELHO" && analiseRisco.statusTexto == "RUPTURA" && !simulacaoAtiva)
    {
      contagemRetornoRuptura++;
      aguardandoResetRuptura = true;
      analiseRisco.recomendacao = "RETORNO DE RUPTURA - Aguardando confirmacao (" + String(contagemRetornoRuptura) + "/3 leituras seguras | Umidade atual: " + String(dadosLocais.umidadeSolo, 1) + "% < " + String(UMIDADE_RUPTURA, 0) + "%)";
      analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;
      Serial.printf("Retorno de ruptura - leitura %d/3\n", contagemRetornoRuptura);
      if (contagemRetornoRuptura >= 3)
      {
        contagemRetornoRuptura = 0;
        aguardandoResetRuptura = false;
        String bufferTexto = "";
        bufferTexto += "RUPTURA ENCERRADA - Umidade abaixo do limiar por 3 leituras consecutivas\r\n";
        Serial.print(bufferTexto);

        analisarRiscoIntegrado();
      }
    }
    else
    {
      contagemRetornoRuptura = 0;
      aguardandoResetRuptura = false;
      if (!simulacaoAtiva)
      {
        analisarRiscoIntegrado();
      }
    }
    ultimaLeituraSensor = agora;
  }

  // Dados BNDMET — retry acelerado se a última consulta falhou
  {
    unsigned long intervaloBndmet = dadosBNDMET.apiDisponivel ? obterIntervaloBNDMET() : INTERVALO_RETRY_API_FALHA;
    if (wifiConectado && (agora - ultimaLeituraBNDMET >= intervaloBndmet))
    {
      garantirWiFi();
      buscarDadosDiarios_I006();
      buscarDadosHorarios_I175();
      ultimaLeituraBNDMET = agora;
    }
  }

  // OpenWeatherMap — retry acelerado se nunca teve sucesso
  {
    bool owmDisponivel = (dadosMeteo.timestamp > 0);
    unsigned long intervaloOwm = owmDisponivel ? INTERVALO_METEO : INTERVALO_RETRY_API_FALHA;
    if (wifiConectado && (agora - ultimaOWM >= intervaloOwm))
    {
      garantirWiFi();
      bool weatherOk = buscarWeatherAtual();
      bool forecastOk = buscarForecast();
      if (!weatherOk && !forecastOk)
        dadosMeteo.timestamp = 0;
      ultimaOWM = agora;
    }
  }

  // Análise de risco periódica — [AJUSTE] agora acompanha o mesmo intervalo
  // adaptativo da leitura do sensor (obterIntervaloSensor()), em vez do
  // antigo valor fixo de 10s. Isso mantém a análise sempre sincronizada com
  // a frequência real de amostragem do higrômetro em cada nível de alerta.
  if (agora - ultimaAnalise >= obterIntervaloSensor())
  {
    if (!simulacaoAtiva && dadosLocais.umidadeSolo < UMIDADE_RUPTURA)
    {
      analisarRiscoIntegrado();
    }
    ultimaAnalise = agora;
  }

  // Envio para API — [IMPORTANTE] agora também decrementa a simulação
  if (agora - ultimoEnvioAPI >= obterIntervaloEnvioAPI())
  {
    if (wifiConectado)
    {
      bool ok = enviarDadosParaAPI();
      if (!ok && tentativasEnvioAPI > 5)
      {
        String bufferTexto = "";
        bufferTexto += "Muitas falhas - verificando conectividade...\r\n";
        Serial.print(bufferTexto);

        verificarConectividade();
      }
      if (simulacaoAtiva && ok)
      {
        simulacaoEnviosRestantes--;

        String bufferTexto = "";
        bufferTexto += "Simulacao: " + String(simulacaoEnviosRestantes) + " envio(s) restante(s)\r\n";
        Serial.print(bufferTexto);

        if (simulacaoEnviosRestantes <= 0)
        {
          simulacaoAtiva = false;
          modoManual = false;
          buzzerAtivo = false;
          noTone(PIN_BUZZER);

          bufferTexto = "";
          bufferTexto += "Simulacao encerrada - retornando ao modo automatico\r\n";
          Serial.print(bufferTexto);
        }
      }
    }

    ultimoEnvioAPI = agora;
  }

  ativarAlarmeBuzzer();

  // Comandos via Serial
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "status")
      mostrarStatusConectividade();
    else if (cmd == "analise")
      analisarRiscoIntegrado();
    else if (cmd == "debug")
      mostrarAnaliseDetalhada();
    else if (cmd == "calibrar")
      recalibrarSensor();
    else if (cmd == "reset")
      resetarSistema();
    else if (cmd == "help")
      mostrarComandosDisponiveis();
    else if (cmd == "teste")
      executarTesteCompleto();
    else if (cmd == "enviar")
    {
      String bufferTexto = "";
      bufferTexto += "Forcando envio...\r\n";
      Serial.print(bufferTexto);

      bool resultadoEnvio = enviarDadosParaAPI();

      bufferTexto = "";
      if (resultadoEnvio)
      {
        bufferTexto += "OK\r\n";
      }
      else
      {
        bufferTexto += "Falha\r\n";
      }
      Serial.print(bufferTexto);
    }
    else if (cmd == "api")
    {
      bool resultadoAPI = verificarStatusAPI();

      String bufferTexto = "";
      if (resultadoAPI)
      {
        bufferTexto += "API OK\r\n";
      }
      else
      {
        bufferTexto += "API falha\r\n";
      }
      Serial.print(bufferTexto);
    }

    // [TESTE] Chuva sintética — toggle em runtime, não altera compilação nem
    // o fluxo real quando desativada (chuvaSinteticaAtiva = false é o padrão).
    else if (cmd == "chuva sintetica")
    {
      chuvaSinteticaAtiva = true;
      String bufferTexto = "";
      bufferTexto += "🌧️ [TESTE] Chuva sintetica ATIVADA\r\n";
      bufferTexto += "   precip24h=40mm | precip7d=100mm | precip30d=200mm | previsao24h=10mm (Moderada)\r\n";
      bufferTexto += "   V_lencol, V_taxa e V_pressao continuam vindos do sensor/OWM reais.\r\n";
      bufferTexto += "   Sweep sugerido de umidade: 13-23% -> AMARELO | 24-29% -> VERMELHO (amplificacao) | >=30% -> RUPTURA\r\n";
      Serial.print(bufferTexto);
    }
    else if (cmd == "chuva real")
    {
      chuvaSinteticaAtiva = false;

      String bufferTexto = "";
      bufferTexto += "[TESTE] Chuva sintetica DESATIVADA - retomando dados reais do BNDMET/OWM\r\n";
      Serial.print(bufferTexto);
    }

    // Comandos de simulação de nível — [IMPORTANTE] agora enviam à API
    else if (cmd == "alerta verde")
    {
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 0;
      dadosLocais.umidadeSolo = 5.0f;
      dadosLocais.fatorLocal = 5.0f / UMIDADE_CRITICA;
      analiseRisco.nivelAlerta = "VERDE";
      analiseRisco.statusTexto = "SEGURO";
      analiseRisco.riscoIntegrado = 0.30f;
      analiseRisco.indiceRisco = 30;
      analiseRisco.amplificado = false;
      analiseRisco.recomendacao = "[Simulacao] NORMAL - Nivel VERDE forcado via comando serial";
      controlarSistemaFisico();
      String bufferTexto = "";
      bufferTexto += "Nivel VERDE (Risco: 30%) [Simulacao]\r\n";
      Serial.print(bufferTexto);

      if (enviarDadosParaAPI())
      {
        simulacaoEnviosRestantes--;
        Serial.printf("Simulacao: %d envio(s) restante(s)\n", simulacaoEnviosRestantes);
      }
    }
    else if (cmd == "alerta amarelo")
    {
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 1;
      dadosLocais.umidadeSolo = 18.0f;
      dadosLocais.fatorLocal = 18.0f / UMIDADE_CRITICA;
      analiseRisco.nivelAlerta = "AMARELO";
      analiseRisco.statusTexto = "ATENÇÃO";
      analiseRisco.riscoIntegrado = 0.60f;
      analiseRisco.indiceRisco = 60;
      analiseRisco.amplificado = false;
      analiseRisco.recomendacao = "[Simulacao] ATENCAO - Nivel AMARELO forcado via comando serial";
      controlarSistemaFisico();
      String bufferTexto = "";
      bufferTexto += "Nivel AMARELO (Risco: 60%) [Simulacao]\r\n";
      Serial.print(bufferTexto);

      if (enviarDadosParaAPI())
      {
        simulacaoEnviosRestantes--;

        String bufferTexto = "";
        bufferTexto += "Simulacao: " + String(simulacaoEnviosRestantes) + " envio(s) restante(s)\r\n";
        Serial.print(bufferTexto);
      }
    }
    else if (cmd == "alerta vermelho")
    {
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 2;
      dadosLocais.umidadeSolo = 27.0f;
      dadosLocais.fatorLocal = 27.0f / UMIDADE_CRITICA;
      analiseRisco.nivelAlerta = "VERMELHO";
      analiseRisco.statusTexto = "CRÍTICO";
      analiseRisco.riscoIntegrado = 0.85f;
      analiseRisco.indiceRisco = 85;
      analiseRisco.amplificado = false;
      buzzerAtivo = true;
      analiseRisco.recomendacao = "[Simulacao] CRITICO - Nivel VERMELHO forcado via comando serial";
      controlarSistemaFisico();
      String bufferTexto = "";
      bufferTexto += "Nivel VERMELHO (Risco: 85%) [Simulacao]\r\n";
      Serial.print(bufferTexto);

      if (enviarDadosParaAPI())
      {
        simulacaoEnviosRestantes--;

        bufferTexto = "";
        bufferTexto += "Simulacao: " + String(simulacaoEnviosRestantes) + " envio(s) restante(s)\r\n";
        Serial.print(bufferTexto);
      }
    }
  }
}
