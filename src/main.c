#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

#include <pwm_z42.h>

#define TPM_DISPARO             TPM2
#define CANAL_DISPARO           0
#define PORTA_DISPARO           GPIOB
#define PINO_DISPARO            2

#define TPM_RETORNO             TPM1
#define CANAL_RETORNO           1
#define PORTA_RETORNO           GPIOB
#define PINO_RETORNO            1

#define IRQ_RETORNO             TPM1_IRQn
#define PRIORIDADE_IRQ          1

#define MODULO_TPM_DISPARO      10000
#define MODULO_TPM_RETORNO      65535

#define CLOCK_TPM_HZ            48000000.0f
#define DIVISOR_RETORNO         128.0f

#define TEMPO_PULSO_US          10
#define INTERVALO_LEITURA_MS    300
#define TEMPO_LIMITE_MS         50

#define VELOCIDADE_SOM_CM_US    0.0343f

static volatile uint16_t borda_inicial = 0;
static volatile uint16_t borda_final = 0;

static volatile bool medida_disponivel = false;
static volatile bool aguardando_descida = false;

static void interrupcao_retorno(void *arg)
{
    ARG_UNUSED(arg);

    TPM_RETORNO->STATUS |= TPM_STATUS_CH1F_MASK;

    if (aguardando_descida == false) {
        borda_inicial = TPM_RETORNO->CONTROLS[CANAL_RETORNO].CnV;
        aguardando_descida = true;
    } else {
        borda_final = TPM_RETORNO->CONTROLS[CANAL_RETORNO].CnV;
        aguardando_descida = false;
        medida_disponivel = true;
    }
}

static void preparar_saida_disparo(void)
{
    pwm_tpm_Init(TPM_DISPARO,
                 TPM_PLLFLL,
                 MODULO_TPM_DISPARO,
                 1,
                 PS_8,
                 EDGE_PWM);

    pwm_tpm_Ch_Init(TPM_DISPARO,
                    CANAL_DISPARO,
                    TPM_PWM_H,
                    PORTA_DISPARO,
                    PINO_DISPARO);

    pwm_tpm_CnV(TPM_DISPARO, CANAL_DISPARO, 0);
}

static void preparar_entrada_retorno(void)
{
    pwm_tpm_Init(TPM_RETORNO,
                 TPM_PLLFLL,
                 MODULO_TPM_RETORNO,
                 1,
                 PS_128,
                 EDGE_PWM);

    pwm_tpm_Ch_Init(TPM_RETORNO,
                    CANAL_RETORNO,
                    TPM_INPUT_CAPTURE_BOTH | TPM_CHANNEL_INTERRUPT,
                    PORTA_RETORNO,
                    PINO_RETORNO);

    IRQ_CONNECT(IRQ_RETORNO,
                PRIORIDADE_IRQ,
                interrupcao_retorno,
                NULL,
                0);

    irq_enable(IRQ_RETORNO);
}

static void reiniciar_medida(void)
{
    medida_disponivel = false;
    aguardando_descida = false;
    borda_inicial = 0;
    borda_final = 0;
}

static void emitir_disparo(void)
{
    reiniciar_medida();

    pwm_tpm_CnV(TPM_DISPARO, CANAL_DISPARO, MODULO_TPM_DISPARO / 1000);
    k_usleep(TEMPO_PULSO_US);
    pwm_tpm_CnV(TPM_DISPARO, CANAL_DISPARO, 0);
}

static bool recebeu_resposta(void)
{
    int tempo_passado;

    for (tempo_passado = 0; tempo_passado < TEMPO_LIMITE_MS; tempo_passado++) {
        if (medida_disponivel) {
            return true;
        }

        k_msleep(1);
    }

    return false;
}

static uint16_t largura_em_ticks(void)
{
    if (borda_final >= borda_inicial) {
        return borda_final - borda_inicial;
    }

    return (uint16_t)((MODULO_TPM_RETORNO - borda_inicial) + borda_final + 1);
}

static float converter_ticks_para_us(uint16_t ticks)
{
    return ticks * ((DIVISOR_RETORNO * 1000000.0f) / CLOCK_TPM_HZ);
}

static float calcular_distancia_cm(float tempo_us)
{
    return (tempo_us * VELOCIDADE_SOM_CM_US) / 2.0f;
}

void main(void)
{
    preparar_saida_disparo();
    preparar_entrada_retorno();

    printk("Modulo ultrassonico inicializado\n");

    while (1) {
        emitir_disparo();

        if (recebeu_resposta()) {
            uint16_t ticks = largura_em_ticks();
            float duracao_us = converter_ticks_para_us(ticks);
            float distancia_cm = calcular_distancia_cm(duracao_us);

            printk("Ticks medidos: %u\n", ticks);
            printk("Distancia medida: %.0f cm\n", distancia_cm);
        } else {
            printk("Sem resposta do echo\n");
        }

        k_msleep(INTERVALO_LEITURA_MS);
    }
}