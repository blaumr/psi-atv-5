#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

#include <pwm_z42.h>

/*
 * Atividade 5 - Sensor ultrassônico
 *
 * Trigger:
 * TPM2_CH0 -> PTB2
 *
 * Echo:
 * TPM1_CH1 -> PTB1
 *
 * Funcionamento:
 * 1. Gera um pulso de trigger.
 * 2. Mede a largura do pulso de echo usando input capture.
 * 3. Converte o tempo medido em distância.
 */

#define TRIGGER_TPM             TPM2
#define TRIGGER_CHANNEL         0
#define TRIGGER_PORT            GPIOB
#define TRIGGER_PIN             2

#define ECHO_TPM                TPM1
#define ECHO_CHANNEL            1
#define ECHO_PORT               GPIOB
#define ECHO_PIN                1

#define ECHO_IRQ_LINE           TPM1_IRQn
#define ECHO_IRQ_PRIORITY       1

#define TRIGGER_TPM_MODULE      10000
#define ECHO_TPM_MODULE         65535

#define TPM_CLOCK_HZ            48000000.0f
#define ECHO_PRESCALER          128.0f

#define TRIGGER_PULSE_US        10
#define MEASUREMENT_PERIOD_MS   300
#define ECHO_TIMEOUT_MS         50

#define SOUND_SPEED_CM_PER_US   0.0343f

volatile uint16_t echo_rising_edge = 0;
volatile uint16_t echo_falling_edge = 0;

volatile bool echo_pulse_ready = false;
volatile bool waiting_for_falling_edge = false;

static void echo_capture_isr(void *arg)
{
    ARG_UNUSED(arg);

    /*
     * Limpa a flag de interrupção do canal 1 do TPM1.
     */
    TPM1->STATUS |= TPM_STATUS_CH1F_MASK;

    if (!waiting_for_falling_edge) {
        /*
         * Primeira borda capturada: subida do echo.
         */
        echo_rising_edge = TPM1->CONTROLS[ECHO_CHANNEL].CnV;
        waiting_for_falling_edge = true;
    } else {
        /*
         * Segunda borda capturada: descida do echo.
         */
        echo_falling_edge = TPM1->CONTROLS[ECHO_CHANNEL].CnV;
        waiting_for_falling_edge = false;
        echo_pulse_ready = true;
    }
}

static void init_trigger_pwm(void)
{
    /*
     * Configura o TPM2 canal 0 para gerar o sinal de trigger em PTB2.
     */
    pwm_tpm_Init(TRIGGER_TPM,
                 TPM_PLLFLL,
                 TRIGGER_TPM_MODULE,
                 1,
                 PS_8,
                 EDGE_PWM);

    pwm_tpm_Ch_Init(TRIGGER_TPM,
                    TRIGGER_CHANNEL,
                    TPM_PWM_H,
                    TRIGGER_PORT,
                    TRIGGER_PIN);

    /*
     * Mantém o trigger inicialmente em nível baixo.
     */
    pwm_tpm_CnV(TRIGGER_TPM, TRIGGER_CHANNEL, 0);
}

static void init_echo_capture(void)
{
    /*
     * Configura o TPM1 canal 1 para capturar o pulso de echo em PTB1.
     * A captura é feita nas duas bordas: subida e descida.
     */
    pwm_tpm_Init(ECHO_TPM,
                 TPM_PLLFLL,
                 ECHO_TPM_MODULE,
                 1,
                 PS_128,
                 EDGE_PWM);

    pwm_tpm_Ch_Init(ECHO_TPM,
                    ECHO_CHANNEL,
                    TPM_INPUT_CAPTURE_BOTH | TPM_CHANNEL_INTERRUPT,
                    ECHO_PORT,
                    ECHO_PIN);

    IRQ_CONNECT(ECHO_IRQ_LINE,
                ECHO_IRQ_PRIORITY,
                echo_capture_isr,
                NULL,
                0);

    irq_enable(ECHO_IRQ_LINE);
}

static void send_trigger_pulse(void)
{
    /*
     * Reinicia o controle de captura antes de iniciar uma nova medição.
     */
    echo_pulse_ready = false;
    waiting_for_falling_edge = false;
    echo_rising_edge = 0;
    echo_falling_edge = 0;

    /*
     * Gera um pulso de trigger de aproximadamente 10 us.
     */
    pwm_tpm_CnV(TRIGGER_TPM, TRIGGER_CHANNEL, TRIGGER_TPM_MODULE / 1000);
    k_usleep(TRIGGER_PULSE_US);
    pwm_tpm_CnV(TRIGGER_TPM, TRIGGER_CHANNEL, 0);
}

static bool wait_for_echo_pulse(void)
{
    /*
     * Aguarda até 50 ms pelo pulso de echo.
     */
    for (int elapsed_ms = 0; elapsed_ms < ECHO_TIMEOUT_MS; elapsed_ms++) {
        if (echo_pulse_ready) {
            return true;
        }

        k_msleep(1);
    }

    return false;
}

static uint16_t calculate_pulse_ticks(void)
{
    /*
     * Calcula a largura do pulso em ticks.
     * Considera também a possibilidade de overflow do contador de 16 bits.
     */
    if (echo_falling_edge >= echo_rising_edge) {
        return echo_falling_edge - echo_rising_edge;
    }

    return (uint16_t)((ECHO_TPM_MODULE - echo_rising_edge) + echo_falling_edge + 1);
}

static float ticks_to_microseconds(uint16_t ticks)
{
    /*
     * tempo_tick = prescaler / clock
     *
     * Em microssegundos:
     * tempo_us = ticks * prescaler * 1e6 / clock
     */
    return ticks * (ECHO_PRESCALER * 1000000.0f / TPM_CLOCK_HZ);
}

static float pulse_us_to_distance_cm(float pulse_width_us)
{
    /*
     * O pulso de echo representa o tempo de ida e volta da onda sonora.
     * Por isso, a distância é dividida por 2.
     */
    return (pulse_width_us * SOUND_SPEED_CM_PER_US) / 2.0f;
}

void main(void)
{
    init_trigger_pwm();
    init_echo_capture();

    printk("Sensor ultrassonico iniciado\n");

    while (1) {
        send_trigger_pulse();

        if (wait_for_echo_pulse()) {
            uint16_t pulse_ticks = calculate_pulse_ticks();
            float pulse_width_us = ticks_to_microseconds(pulse_ticks);
            float distance_cm = pulse_us_to_distance_cm(pulse_width_us);

            printk("Largura de ticks: %u\n", pulse_ticks);
            printk("Distancia: %.0f cm\n", distance_cm);
        } else {
            printk("Echo nao recebido\n");
        }

        k_msleep(MEASUREMENT_PERIOD_MS);
    }
}