// #include <stdio.h> // Biblioteca estándar para entrada/salida
// #include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS
// #include "freertos/task.h" // Soporte para tareas
// #include "freertos/queue.h" // Soporte para colas
// #include "driver/ledc.h" // Controlador PWM para LED
// #include "driver/gpio.h" // Controlador para pines GPIO
// #include "esp_err.h" // Manejo de errores
// #include "esp_intr_alloc.h" // Interrupciones
// #include "esp_timer.h" // Temporizador de precisión para debounce
// #include "rgb.h"   // Librería personalizada 


// // Pines conectados al LED RGB
// #define LED_R_GPIO 18
// #define LED_G_GPIO 19
// #define LED_B_GPIO 21

// // Pin conectado al botón
// #define BUTTON_GPIO 0

// // Configuración del PWM (LEDC)
// #define LEDC_TIMER LEDC_TIMER_0
// #define LEDC_MODE LEDC_HIGH_SPEED_MODE
// #define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolución de 13 bits: 8192 niveles
// #define LEDC_FREQUENCY 5000 // Frecuencia de PWM
// #define MAX_DUTY ((1 << LEDC_DUTY_RES) - 1) // 8191 (valor máximo duty)

// // Canales PWM para cada color
// #define LEDC_CHANNEL_R LEDC_CHANNEL_0
// #define LEDC_CHANNEL_G LEDC_CHANNEL_1
// #define LEDC_CHANNEL_B LEDC_CHANNEL_2

// // Variables globales
// static uint8_t brightness_level = 0; // Índice del nivel actual en el array levels
// static const uint8_t levels[] = {0, 25, 50, 75, 100}; // Niveles de brillo
// static QueueHandle_t gpio_evt_queue = NULL; // Cola para manejar eventos del botón

// // Variables para debounce
// static int64_t last_press_time = 0; // Almacena el tiempo de la última pulsación
// #define DEBOUNCE_TIME_US 200000 // 200 milisegundos

// // ---------- BLOQUE 1: INTERRUPCIÓN ----------
// // ISR que se ejecuta cuando se presiona el botón
// static void IRAM_ATTR gpio_isr_handler(void* arg) {
//     int64_t now = esp_timer_get_time();
//     if (now - last_press_time > DEBOUNCE_TIME_US) {
//         last_press_time = now;
//         uint32_t gpio_num = (uint32_t) arg;
//         xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); // Enviar evento a la cola
//     }
// }

// // ---------- BLOQUE 2: PWM (LEDC) ----------
// // Configura los tres canales de PWM para el LED RGB
// void ledc_configure(void) {
//     // Configurar temporizador del PWM
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode = LEDC_MODE,
//         .timer_num = LEDC_TIMER,
//         .duty_resolution = LEDC_DUTY_RES,
//         .freq_hz = LEDC_FREQUENCY,
//         .clk_cfg = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);

//     // Configurar cada canal para R, G y B
//     ledc_channel_config_t ledc_channel[3] = {
//         {
//             .channel = LEDC_CHANNEL_R,
//             .duty = 0,
//             .gpio_num = LED_R_GPIO,
//             .speed_mode = LEDC_MODE,
//             .hpoint = 0,
//             .timer_sel = LEDC_TIMER
//         },
//         {
//             .channel = LEDC_CHANNEL_G,
//             .duty = 0,
//             .gpio_num = LED_G_GPIO,
//             .speed_mode = LEDC_MODE,
//             .hpoint = 0,
//             .timer_sel = LEDC_TIMER
//         },
//         {
//             .channel = LEDC_CHANNEL_B,
//             .duty = 0,
//             .gpio_num = LED_B_GPIO,
//             .speed_mode = LEDC_MODE,
//             .hpoint = 0,
//             .timer_sel = LEDC_TIMER
//         },
//     };

//     // Aplicar configuración a cada canal
//     for (int ch = 0; ch < 3; ch++) {
//         ledc_channel_config(&ledc_channel[ch]);
//     }
// }


// void update_led_duty(uint32_t percent) {
//     uint32_t duty = MAX_DUTY * percent / 100; // Convertir porcentaje a valor de 13 bits

//     // Invertir el duty para LED de ánodo común
//     uint32_t inverted_duty = MAX_DUTY - duty;  // Valor invertido

//     // Asignar duty invertido a cada color con distintas proporciones para crear mezcla
//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, inverted_duty);
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);

//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, inverted_duty );
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G);

//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, inverted_duty );
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
// }



// // ---------- BLOQUE 3: BOTÓN ----------
// // Configuración del botón como entrada con interrupción
// void gpio_button_configure(void) {
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_POSEDGE, // Detectar flanco de subida
//         .mode = GPIO_MODE_INPUT, // Modo entrada
//         .pin_bit_mask = (1ULL << BUTTON_GPIO),
//         .pull_down_en = 0,
//         .pull_up_en = 1 // Activamos pull-up interna
//     };
//     gpio_config(&io_conf);

//     // Crear cola de eventos y configurar ISR
//     gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);
// }

// // ---------- BLOQUE 4: FUNCIÓN PRINCIPAL ----------
// // Ciclo principal del programa
// void app_main(void) {
//     ledc_configure(); // Configurar LED PWM
//     gpio_button_configure(); // Configurar botón con interrupciones

//     uint32_t io_num;
//     while (1) {
//         // Esperar evento del botón (interrupción)
//         update_led_duty(levels[brightness_level]); // Asignar el nuevo duty
//         if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
//             // Avanzar al siguiente nivel de brillo
//             brightness_level = (brightness_level + 1) % 5; // Cicla entre 0 y 4
//             update_led_duty(levels[brightness_level]); // Asignar el nuevo duty
//             printf("Brillo: %d\n", levels[brightness_level]); // Imprimir nivel de brillo

//         }
//     }
// }


#include <stdio.h> // Biblioteca estándar para entrada/salida
#include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS
#include "freertos/task.h" // Soporte para tareas
#include "freertos/queue.h" // Soporte para colas
#include "driver/ledc.h" // Controlador PWM para LED
#include "driver/gpio.h" // Controlador para pines GPIO
#include "esp_err.h" // Manejo de errores
#include "esp_intr_alloc.h" // Interrupciones
#include "esp_timer.h" // Temporizador de precisión para debounce
#include "rgb.h"   // Librería personalizada 


// Pines conectados al LED RGB
#define LED_R_GPIO 18
#define LED_G_GPIO 19
#define LED_B_GPIO 21

// Pin conectado al botón
#define BUTTON_GPIO 0

// Configuración del PWM (LEDC)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolución de 13 bits: 8192 niveles
#define LEDC_FREQUENCY 5000 // Frecuencia de PWM
#define MAX_DUTY ((1 << LEDC_DUTY_RES) - 1) // 8191 (valor máximo duty)

// Canales PWM para cada color
#define LEDC_CHANNEL_R LEDC_CHANNEL_0
#define LEDC_CHANNEL_G LEDC_CHANNEL_1
#define LEDC_CHANNEL_B LEDC_CHANNEL_2

// Variables globales
static uint8_t brightness_level = 0; // Índice del nivel actual en el array levels
static const uint8_t levels[] = {0, 25, 50, 75, 100}; // Niveles de brillo
static QueueHandle_t gpio_evt_queue = NULL; // Cola para manejar eventos del botón

// Variables para debounce
static int64_t last_press_time = 0; // Almacena el tiempo de la última pulsación
#define DEBOUNCE_TIME_US 200000 // 200 milisegundos

// ---------- BLOQUE 1: INTERRUPCIÓN ----------
// ISR que se ejecuta cuando se presiona el botón
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_time > DEBOUNCE_TIME_US) {
        last_press_time = now;
        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); // Enviar evento a la cola
    }
}

// ---------- BLOQUE 2: PWM (LEDC) ----------
// Configura los tres canales de PWM para el LED RGB
void ledc_configure(void) {
    // Configurar temporizador del PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configurar cada canal para R, G y B
    ledc_channel_config_t ledc_channel[3] = {
        {
            .channel = LEDC_CHANNEL_R,
            .duty = 0,
            .gpio_num = LED_R_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER
        },
        {
            .channel = LEDC_CHANNEL_G,
            .duty = 0,
            .gpio_num = LED_G_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER
        },
        {
            .channel = LEDC_CHANNEL_B,
            .duty = 0,
            .gpio_num = LED_B_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER
        },
    };

    // Aplicar configuración a cada canal
    for (int ch = 0; ch < 3; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
}


void update_led_duty(uint32_t percent) {
    uint32_t duty = MAX_DUTY * percent / 100; // Convertir porcentaje a valor de 13 bits

    // Invertir el duty para LED de ánodo común
    uint32_t inverted_duty = MAX_DUTY - duty;  // Valor invertido

    // Asignar duty invertido a cada color con distintas proporciones para crear mezcla
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, inverted_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, inverted_duty );
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, inverted_duty );
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}



// ---------- BLOQUE 3: BOTÓN ----------
// Configuración del botón como entrada con interrupción
void gpio_button_configure(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Detectar flanco de subida
        .mode = GPIO_MODE_INPUT, // Modo entrada
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1 // Activamos pull-up interna
    };
    gpio_config(&io_conf);

    // Crear cola de eventos y configurar ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);
}

// ---------- BLOQUE 4: FUNCIÓN PRINCIPAL ----------
// Ciclo principal del programa
void app_main(void) {
    ledc_configure(); // Configurar LED PWM
    gpio_button_configure(); // Configurar botón con interrupciones

    uint32_t io_num;
    while (1) {
        saludar(); // Llamar a la función de saludo
        // Esperar evento del botón (interrupción)
        update_led_duty(levels[brightness_level]); // Asignar el nuevo duty
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // Avanzar al siguiente nivel de brillo
            brightness_level = (brightness_level + 1) % 5; // Cicla entre 0 y 4
            update_led_duty(levels[brightness_level]); // Asignar el nuevo duty
            printf("Brillo: %d\n", levels[brightness_level]); // Imprimir nivel de brillo

        }
    }
}


