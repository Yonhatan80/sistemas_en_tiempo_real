#include <stdio.h> // Biblioteca estándar para entrada/salida
#include "rgb.h"

void saludar(void) {
    printf("Hola desde la librería!\n");
}

int sumar(int a, int b) {
    return a + b;
}

//añadido
// void configurar_led_pwm(ledc_channel_t canal, uint32_t duty, int gpio_num) {
//     ledc_channel_config_t config = {
//         .channel    = canal,
//         .duty       = duty,
//         .gpio_num   = gpio_num,
//         .speed_mode = LEDC_MODE,
//         .hpoint     = 0,
//         .timer_sel  = LEDC_TIMER
//     };

//     ledc_channel_config(&config);
// }