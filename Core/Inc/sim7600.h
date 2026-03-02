/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sim7600.h
  * @brief   SIM7600G AT command driver — LPUART1 + circular DMA RX
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SIM7600_H
#define SIM7600_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Buffer sizes ──────────────────────────────────────────────────────────── */
#define SIM7600_DMA_BUF_SIZE    512u    /* Circular DMA RX buffer (power of 2) */
#define SIM7600_RESP_BUF_SIZE   512u    /* Accumulated response buffer          */

/* ── Default timeouts (ms) ─────────────────────────────────────────────────── */
#define SIM7600_TIMEOUT_SHORT   1000u   /* AT, ATI, echo off, etc.             */
#define SIM7600_TIMEOUT_MEDIUM  5000u   /* SIM / network status queries        */
#define SIM7600_TIMEOUT_LONG   30000u   /* Registration, attach, dial          */

/* ── Status codes ──────────────────────────────────────────────────────────── */
typedef enum {
    SIM7600_OK      = 0,   /* Expected response found   */
    SIM7600_ERROR   = 1,   /* Modem replied with ERROR  */
    SIM7600_TIMEOUT = 2,   /* No response within timeout */
} SIM7600_Status;

/* ── PWRKEY pin (ARD_D9 on P-L496G-CELL02 / DFRobot shield) ───────────────── */
#define SIM7600_PWRKEY_PORT     ARD_D9_GPIO_Port   /* GPIOH */
#define SIM7600_PWRKEY_PIN      ARD_D9_Pin          /* GPIO_PIN_13 */

/* ── Public API ────────────────────────────────────────────────────────────── */

/**
 * @brief  Pulse PWRKEY to power on the SIM7600G-H and wait for it to boot.
 *
 *         PWRKEY sequence (per SIM7600 HW spec + DFRobot shield schematic):
 *           D9 HIGH → NPN transistor on → PWRKEY pulled LOW for 500 ms
 *           D9 LOW  → PWRKEY released
 *           Wait 8 s for the module to boot and emit "RDY"
 *
 *         Call this BEFORE SIM7600_Init(). The ARD_D9 GPIO must already be
 *         configured as output push-pull (done by MX_GPIO_Init).
 */
void SIM7600_PowerOn(void);

/**
 * @brief  Initialise the driver and start circular DMA receive.
 *         Call once after MX_LPUART1_UART_Init() and MX_DMA_Init().
 * @param  huart  Pointer to the LPUART1 handle (hlpuart1).
 */
void SIM7600_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Repeatedly send "AT" until the modem responds or total_ms elapses.
 *         Mirrors TinyGSM's testAT() — handles auto-baud sync and late boot.
 *         Each attempt uses a 200 ms window; attempts are spaced 100 ms apart.
 *
 * @param  total_ms  Total time budget (10000 is a sensible default).
 * @retval SIM7600_OK if the modem responded with "OK", SIM7600_TIMEOUT otherwise.
 */
SIM7600_Status SIM7600_TestAT(uint32_t total_ms);

/**
 * @brief  Send an AT command and wait for an expected response string.
 *
 * @param  cmd          Suffix after "AT", e.g. "+CSQ" → sends "AT+CSQ\r\n".
 *                      Pass NULL or "" to send a bare "AT\r\n".
 * @param  expected     String to look for in the response, e.g. "OK", "+CSQ".
 * @param  resp_buf     Optional buffer to copy the full modem response into.
 *                      Pass NULL if you don't need the raw response.
 * @param  resp_buf_len Size of resp_buf (including null terminator).
 * @param  timeout_ms   How long to wait before giving up.
 * @retval SIM7600_OK / SIM7600_ERROR / SIM7600_TIMEOUT
 */
SIM7600_Status SIM7600_SendAT(const char *cmd,
                               const char *expected,
                               char       *resp_buf,
                               uint16_t    resp_buf_len,
                               uint32_t    timeout_ms);

/**
 * @brief  Must be called from HAL_UARTEx_RxEventCallback().
 *         Drains newly arrived DMA bytes into the internal response buffer.
 *
 * @param  huart  The UART handle forwarded from the HAL callback.
 * @param  size   The Size parameter forwarded from the HAL callback.
 */
void SIM7600_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* SIM7600_H */
