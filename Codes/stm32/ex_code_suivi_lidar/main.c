#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define UART_LIDAR &huart1
#define LIDAR_BUFFER_SIZE 512
#define MAX_SPEED 28.0f
#define V_CONSIGNE 3.0f
#define K 0.5f
#define K_ANGLE 0.5f
#define OBSTACLE_THRESHOLD 1.0f

// Simule tableau de 360 mesures
float lidar_data[360] = {0};

// UART buffer
uint8_t lidar_rx_buffer[LIDAR_BUFFER_SIZE];

// Fonctions prototypes
void set_motor_speed(float left_kmh, float right_kmh);
void parse_lidar_data(uint8_t *data, uint16_t len);
float clamp(float val, float min, float max);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();  // Pour moteurs (PWM)

    printf("RPLidar UART Test - STM32\n");

    while (1)
    {
        // Lire les données du Lidar via UART
        if (HAL_UART_Receive(UART_LIDAR, lidar_rx_buffer, LIDAR_BUFFER_SIZE, HAL_MAX_DELAY) == HAL_OK)
        {
            parse_lidar_data(lidar_rx_buffer, LIDAR_BUFFER_SIZE);

            float delta_x_g = lidar_data[90];
            float delta_x_d = lidar_data[270];
            float delta_x_avant = lidar_data[180];

            printf("G: %.2f | D: %.2f | A: %.2f\n", delta_x_g, delta_x_d, delta_x_avant);

            float angle;
            if (delta_x_avant < OBSTACLE_THRESHOLD)
            {
                angle = K_ANGLE * (delta_x_g - delta_x_d);
            }
            else
            {
                angle = K_ANGLE * (lidar_data[60] - lidar_data[300]);
            }

            float Vg = V_CONSIGNE - K * (delta_x_d - delta_x_g);
            float Vd = V_CONSIGNE + K * (delta_x_g - delta_x_d);

            Vg = clamp(Vg, -MAX_SPEED, MAX_SPEED);
            Vd = clamp(Vd, -MAX_SPEED, MAX_SPEED);

            set_motor_speed(Vg, Vd);
        }
    }
}

float clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// ⚠️ À remplacer avec du vrai parsing si tu veux
void parse_lidar_data(uint8_t *data, uint16_t len)
{
    // Simule des valeurs pour test
    lidar_data[90] = 1.2f;
    lidar_data[180] = 0.8f;
    lidar_data[270] = 1.1f;
    lidar_data[60] = 1.5f;
    lidar_data[300] = 1.4f;
}

// ⚠️ Remplace cette fonction par un vrai contrôle PWM
void set_motor_speed(float left_kmh, float right_kmh)
{
    float pwm_left = (left_kmh / MAX_SPEED) * 100.0f;
    float pwm_right = (right_kmh / MAX_SPEED) * 100.0f;

    pwm_left = clamp(pwm_left, 0.0f, 100.0f);
    pwm_right = clamp(pwm_right, 0.0f, 100.0f);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left);   // Moteur gauche
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);  // Moteur droit
}
