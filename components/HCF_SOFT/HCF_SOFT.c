#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include "esp_rom.h"
#include <stdio.h>
//#include <string.h>
//#include "esp_system.h"


    void delay_us (int us)
    {
        esp_rom_delay_us(us);
    }
    void delay_ms (int ms)
    {
        vTaskDelay(ms / portTICK_PERIOD_MS); 
    }
    void piscar_LED(int vezes, int pino, int tempo_alto, int tempo_baixo)
    {
        for(int i = 0; i < vezes; i++)
        {
            gpio_set_level(pino, 1);
            delay_ms(tempo_alto);
            
            gpio_set_level(pino, 0);
            delay_ms(tempo_baixo);
        }
    }

