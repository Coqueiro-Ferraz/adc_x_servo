//////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                       _              //
//               _    _       _      _        _     _   _   _    _   _   _        _   _  _   _          //
//           |  | |  |_| |\| |_| |\ |_|   |\ |_|   |_| |_| | |  |   |_| |_| |\/| |_| |  |_| | |   /|    //    
//         |_|  |_|  |\  | | | | |/ | |   |/ | |   |   |\  |_|  |_| |\  | | |  | | | |_ | | |_|   _|_   //
//                                                                                       /              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*   Programa básico para controle da placa durante a Jornada da Programação 1
*   Permite o controle das entradas e saídas digitais, entradas analógicas, display LCD e teclado. 
*   Cada biblioteca pode ser consultada na respectiva pasta em componentes
*   Existem algumas imagens e outros documentos na pasta Recursos
*   O código principal pode ser escrito a partir da linha 83
*/

// Área de inclusão das bibliotecas
//-----------------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/ledc.h"
#include "esp_log.h"

#include "HCF_IOTEC.h"   // Vai se tornar HCF_IOTEC
#include "HCF_LCD.h" // Vai se tornar HCF_LCD
#include "HCF_ADC.h"   // Vai se tornar HCF_ADC
#include "HCF_MP.h"   // Vai se tornar HCF_MP
#include "HCF_SOFT.h" 
// Incluir HCF_IOT HCF_BT HCF_DHT HCF_ULTRA HCF_RFID HCF_ZMPT HCF_ACS HCF_SERVO HCF_OLED HCF_CAM HCF_SD HCF_LORA


// Área das macros
//-----------------------------------------------------------------------------------------------------------------------

#define IN(x) (entradas>>x)&1
#define OUT(x) (1 << (x))

// ADC config
#define ADC_UNIT       ADC_UNIT_1
#define ADC_CHANNEL    ADC_CHANNEL_0   //
#define ADC_ATTEN      ADC_ATTEN_DB_12

// Servo config
#define SERVO_GPIO     26
#define SERVO_FREQ_HZ  50
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500

// Pot config
#define VREF           3.3
#define ADC_MAX        4095.0
#define POT_MAX_ANGLE  270.0

// LEDC config
#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_RES       LEDC_TIMER_16_BIT


// Área de declaração de variáveis e protótipos de funções
//-----------------------------------------------------------------------------------------------------------------------

static const char *TAG = "Placa";
static uint8_t entradas, saidas = 0; //variáveis de controle de entradas e saídas
static char tecla = '-' ;
char escrever[40];


static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

// Funções e ramos auxiliares
//-----------------------------------------------------------------------------------------------------------------------
// ---------------- SERVO ----------------
void servo_init()
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

// Converte microsegundos para duty
uint32_t us_to_duty(uint32_t us)
{
    uint32_t max_duty = (1 << LEDC_RES) - 1;
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;

    return (us * max_duty) / period_us;
}

void servo_set_angle(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Mapeia ângulo → pulso
    float pulse = SERVO_MIN_US +
                  (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US);

    uint32_t duty = us_to_duty((uint32_t)pulse);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// ---------------- ADC ----------------
void adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);

    // calibração
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
}

// Programa Principal
//-----------------------------------------------------------------------------------------------------------------------

void app_main(void)
{
    /////////////////////////////////////////////////////////////////////////////////////   Programa principal
    escrever[39] = '\0';

    // a seguir, apenas informações de console, aquelas notas verdes no início da execução
    ESP_LOGI(TAG, "Iniciando...");
    ESP_LOGI(TAG, "Versão do IDF: %s", esp_get_idf_version());



    /////////////////////////////////////////////////////////////////////////////////////   Inicializações de periféricos (manter assim)
    
    // inicializar os IOs e teclado da placa
    iniciar_iotec();      
    entradas = io_le_escreve(saidas); // Limpa as saídas e lê o estado das entradas

    adc_init();
    servo_init();

    // inicializar o display LCD 
    iniciar_lcd();
    escreve_lcd(1,0,"Angulo do pot   ");
    escreve_lcd(2,0,"Controle   servo");
    
    // Inicializar o componente de leitura de entrada analógica
    /*esp_err_t init_result = iniciar_adc_CHX(0);
    if (init_result != ESP_OK) {
        ESP_LOGE("MAIN", "Erro ao inicializar o componente ADC personalizado");
    }*/

    //delay inicial
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    limpar_lcd();

    /////////////////////////////////////////////////////////////////////////////////////   Periféricos inicializados

    piscar_LED(3,2,100,100);

    /////////////////////////////////////////////////////////////////////////////////////   Início do ramo principal                    
    while (1)                                                                                                                         
    {                                                                                                                                 
        //_______________________________________________________________________________________________________________________________________________________ //
        //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -  -  -  -  -  -  -  -  -  -  Escreva seu código aqui!!! //

        int raw = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);

        int voltage_mv = 0;
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);

        float voltage = voltage_mv / 1000.0;

        // ângulo do potenciômetro (0–270° típico)
        float pot_angle = (raw / ADC_MAX) * POT_MAX_ANGLE;

        // servo (limitado a 180°)
        float servo_angle = ((float)raw / ADC_MAX) * 180.0;

        servo_set_angle(servo_angle);

        ESP_LOGI(TAG,
                 "ADC: %d | V: %.2f V | Pot: %.1f° | Servo: %.1f°",
                 raw, voltage, pot_angle, servo_angle);

    


        sprintf(escrever,"AD: %d   ", raw);
        escreve_lcd(1,0,escrever);

        sprintf(escrever,"V: %.2fV   ", voltage);
        escreve_lcd(2,0,escrever);

        sprintf(escrever,"Pot: %.1f\'   ",pot_angle);
        escreve_lcd(1,20,escrever);

        sprintf(escrever,"Servo: %.1f\'   ", servo_angle);
        escreve_lcd(2,20,escrever);

        vTaskDelay(pdMS_TO_TICKS(200)); 
        
        //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -  -  -  -  -  -  -  -  -  -  Escreva seu só até aqui!!! //
        //________________________________________________________________________________________________________________________________________________________//

    }
    
    // caso erro no programa, desliga o módulo ADC
//    adc_limpar();

    /////////////////////////////////////////////////////////////////////////////////////   Fim do ramo principal
    
}
