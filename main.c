
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// =====================================================
// DEFINICAO DOS GPIOs
// =====================================================

#define GPIO_ROTACAO GPIO_NUM_4
#define GPIO_LED GPIO_NUM_2
#define GPIO_INJETOR GPIO_NUM_25
#define GPIO_BOMBA GPIO_NUM_27

#define GPIO_TEMP_AR GPIO_NUM_32
#define GPIO_TEMP_AGUA GPIO_NUM_33
#define GPIO_MAP GPIO_NUM_34
#define GPIO_TPS GPIO_NUM_35

#define RPM_LINHAS 25
#define MAP_COLUNAS 19
#define CORRECOES 11

static const char *TAG = "ECU";

// =====================================================
// MAPA PRINCIPAL DE INJECAO
//
// Linhas: 400 a 10000 RPM, passo de 400 RPM
// Colunas: 0.00, -0.05, ... -0.90
//
// Valores de exemplo em milissegundos.
// =====================================================

static const float mapa_principal[RPM_LINHAS][MAP_COLUNAS] = {
    {1.5, 1.5, 1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.8, 1.9, 2.0, 2.0, 2.1, 2.1, 2.2,
     2.2, 2.3, 2.3, 2.4},
    {1.5, 1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.8, 1.9, 2.0, 2.0, 2.1, 2.2, 2.2, 2.3,
     2.4, 2.4, 2.5, 2.5},
    {1.5, 1.6, 1.6, 1.7, 1.7, 1.8, 1.9, 1.9, 2.0, 2.1, 2.1, 2.2, 2.3, 2.3, 2.4,
     2.5, 2.5, 2.6, 2.6},
    {1.6, 1.6, 1.7, 1.7, 1.8, 1.9, 1.9, 2.0, 2.1, 2.2, 2.2, 2.3, 2.4, 2.5, 2.5,
     2.6, 2.7, 2.7, 2.8},
    {1.6, 1.7, 1.7, 1.8, 1.9, 2.0, 2.0, 2.1, 2.2, 2.3, 2.4, 2.4, 2.5, 2.6, 2.7,
     2.8, 2.8, 2.9, 3.0},
    {1.7, 1.7, 1.8, 1.9, 2.0, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.5, 2.6, 2.7, 2.8,
     2.9, 3.0, 3.0, 3.1},
    {1.7, 1.8, 1.9, 2.0, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0,
     3.1, 3.1, 3.2, 3.3},
    {1.8, 1.9, 2.0, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1,
     3.2, 3.3, 3.4, 3.5},
    {1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2,
     3.3, 3.4, 3.5, 3.6},
    {1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3,
     3.4, 3.5, 3.6, 3.7},
    {2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4,
     3.5, 3.6, 3.7, 3.8},
    {2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4,
     3.5, 3.6, 3.7, 3.8},
    {2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5,
     3.6, 3.7, 3.8, 3.9},
    {2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5,
     3.6, 3.7, 3.8, 3.9},
    {2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6,
     3.7, 3.8, 3.9, 4.0},
    {2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6,
     3.7, 3.8, 3.9, 4.0},
    {2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7,
     3.8, 3.9, 4.0, 4.1},
    {2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7,
     3.8, 3.9, 4.0, 4.1},
    {2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8,
     3.9, 4.0, 4.1, 4.2},
    {2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8,
     3.9, 4.0, 4.1, 4.2},
    {2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9,
     4.0, 4.1, 4.2, 4.3},
    {2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9,
     4.0, 4.1, 4.2, 4.3},
    {2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0,
     4.1, 4.2, 4.3, 4.4},
    {2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0,
     4.1, 4.2, 4.3, 4.4},
    {2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1,
     4.2, 4.3, 4.4, 4.5}};

// =====================================================
// MAPAS DE CORRECAO
// Valores armazenados em porcentagem.
//
// O eixo de referencia de cada tabela e:
// 0, 10, 20, ... 100
// =====================================================

static const float mapa_temp[CORRECOES] = {0,   1.5, 2.0, 2.5, 3.0, 3.5,
                                           4.0, 4.5, 5.0, 5.5, 6.0};

static const float mapa_tps[CORRECOES] = {0,   1.5, 2.0, 2.5, 3.0, 3.5,
                                          4.0, 4.5, 5.0, 5.5, 6.0};

static const float mapa_temp_ar[CORRECOES] = {0,   1.5, 2.0, 2.5, 3.0, 3.5,
                                              4.0, 4.5, 5.0, 5.5, 6.0};

// =====================================================
// SELECAO DO TEMPO DE INJECAO NO MAPA PRINCIPAL
// pressao: eixo relativo de 0.00 a -0.90
// rpm: rotacao atual
// Retorno: tempo-base em milissegundos
// =====================================================

static float calcular_tempo_injecao(float pressao, int rpm) {
  int r;
  int m;

  // Seleciona a faixa de RPM
  if (rpm <= 400) {
    r = 0;
  } else if (rpm >= 10000) {
    r = RPM_LINHAS - 1;
  } else {
    r = (rpm - 1) / 400;
  }

  // Limita a pressao ao eixo do mapa
  if (pressao > 0.0f) {
    pressao = 0.0f;
  }

  if (pressao < -0.90f) {
    pressao = -0.90f;
  }

  // O eixo esta ordenado de 0.00 a -0.90
  for (m = 0; m < MAP_COLUNAS - 1; m++) {
    float pressao_eixo = -(float)m * 0.05f;

    if (pressao >= pressao_eixo) {
      break;
    }
  }

  return mapa_principal[r][m];
}

// =====================================================
// SELECAO DE UMA CORRECAO PERCENTUAL
//
// Valor do sensor entre 0 e 100.
// Cada faixa equivale a 10 unidades.
// =====================================================

static float obter_correcao(float valor_sensor, const float tabela[CORRECOES]) {
  int indice;

  if (valor_sensor < 0.0f) {
    valor_sensor = 0.0f;
  }

  if (valor_sensor > 100.0f) {
    valor_sensor = 100.0f;
  }

  indice = (int)(valor_sensor / 10.0f);

  if (indice >= CORRECOES) {
    indice = CORRECOES - 1;
  }

  return tabela[indice];
}

// =====================================================
// APLICACAO DAS CORRECOES
//
// temp: temperatura do motor
// temp_ar: temperatura do ar
// tps: abertura percentual da borboleta
// tempo_base: valor retornado pelo mapa principal
// =====================================================

static float correcao(float temp, float temp_ar, float tps, float tempo_base) {
  float percentual;

  // Correcao da temperatura do motor
  percentual = obter_correcao(temp, mapa_temp);
  tempo_base *= 1.0f + percentual / 100.0f;

  // Correcao do TPS
  percentual = obter_correcao(tps, mapa_tps);
  tempo_base *= 1.0f + percentual / 100.0f;

  // Correcao da temperatura do ar
  percentual = obter_correcao(temp_ar, mapa_temp_ar);
  tempo_base *= 1.0f + percentual / 100.0f;

  return tempo_base;
}

// =====================================================
// FUNCOES DE LEITURA DOS SENSORES
//
// IMPLEMENTAR: estas funcoes sao apenas pontos de
// integracao. Ainda nao convertem tensao em grandezas
// fisicas e nao medem a rotacao real.
// =====================================================

static int ler_rpm(void) {
  // TODO: calcular RPM usando os pulsos no GPIO 4.
  return 0;
}

static float ler_map(void) {
  // TODO: ler ADC1 e converter para o eixo MAP.
  // Retornar valor entre 0.00 e -0.90.
  return 0.0f;
}

static float ler_tps(void) {
  // TODO: converter a tensao do TPS para 0 a 100%.
  return 0.0f;
}

static float ler_temp(void) {
  // TODO: converter a leitura do NTC em graus Celsius.
  return 0.0f;
}

static float ler_temp_ar(void) {
  // TODO: converter a leitura do NTC em graus Celsius.
  return 0.0f;
}

// =====================================================
// CONFIGURACAO DOS GPIOs DE SAIDA
// =====================================================

static void configurar_gpio(void) {
  gpio_config_t saidas = {.pin_bit_mask = (1ULL << GPIO_LED) |
                                          (1ULL << GPIO_INJETOR) |
                                          (1ULL << GPIO_BOMBA),
                          .mode = GPIO_MODE_OUTPUT,
                          .pull_up_en = GPIO_PULLUP_DISABLE,
                          .pull_down_en = GPIO_PULLDOWN_ENABLE,
                          .intr_type = GPIO_INTR_DISABLE};

  ESP_ERROR_CHECK(gpio_config(&saidas));

  // Estado inicial seguro: atuadores desligados
  gpio_set_level(GPIO_LED, 0);
  gpio_set_level(GPIO_INJETOR, 0);
  gpio_set_level(GPIO_BOMBA, 0);
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================

void app_main(void) {
  configurar_gpio();

  ESP_LOGI(TAG, "ECU iniciada");
  ESP_LOGW(TAG, "Leituras dos sensores ainda nao implementadas");

  while (1) {
    int rpm;
    float pressao;
    float tps;
    float temp_motor;
    float temp_ar;
    float tempo_base;
    float tempo_final;

    // Leitura dos sensores
    rpm = ler_rpm();
    pressao = ler_map();
    tps = ler_tps();
    temp_motor = ler_temp();
    temp_ar = ler_temp_ar();

    // Calcula o tempo-base
    tempo_base = calcular_tempo_injecao(pressao, rpm);

    // Aplica as correcoes
    tempo_final = correcao(temp_motor, temp_ar, tps, tempo_base);

    // Indicacao visual para teste
    gpio_set_level(GPIO_LED, rpm > 0 ? 1 : 0);

    ESP_LOGI(TAG,
             "RPM=%d MAP=%.2f TPS=%.1f Temp=%.1f "
             "IAT=%.1f Base=%.2f ms Final=%.2f ms",
             rpm, pressao, tps, temp_motor, temp_ar, tempo_base, tempo_final);

    // Esta rotina calcula, mas ainda nao dispara o bico.
    // O acionamento real precisa de temporizacao precisa,
    // sincronizada com a rotacao, e driver de potencia.

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}