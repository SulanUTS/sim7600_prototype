#include "sim7600.h"
#include <string.h>
#include <stdio.h>

/* ── Private state ─────────────────────────────────────────────────────────── */

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t  s_dma_buf[SIM7600_DMA_BUF_SIZE];

static uint8_t  s_resp_buf[SIM7600_RESP_BUF_SIZE];
static uint16_t s_resp_len = 0;

static uint16_t s_dma_prev = 0;

/* ── Private helpers ───────────────────────────────────────────────────────── */

/**
 * @brief  Append up to 'len' bytes from src into s_resp_buf, respecting the
 *         buffer limit and keeping it null-terminated.
 */
static void append_resp(const uint8_t *src, uint16_t len)
{
    /* Leave one byte for null terminator */
    uint16_t space = (SIM7600_RESP_BUF_SIZE - 1u) - s_resp_len;
    if (len > space)
        len = space;
    if (len == 0u)
        return;

    memcpy(&s_resp_buf[s_resp_len], src, len);
    s_resp_len += len;
    s_resp_buf[s_resp_len] = '\0';
}

/**
 * @brief  Drain bytes [s_dma_prev .. new_head) out of the circular DMA buffer.
 *         Handles wrap-around correctly.
 *
 * @param  new_head  Current DMA write position (0 .. DMA_BUF_SIZE-1).
 *                   When a TC event fires, pass 0 (buffer wrapped fully).
 */
static void drain_dma(uint16_t new_head)
{
    if (new_head == s_dma_prev)
        return; /* Nothing new */

    if (new_head > s_dma_prev)
    {
        /* Simple case — no wrap */
        append_resp(&s_dma_buf[s_dma_prev], new_head - s_dma_prev);
    }
    else
    {
        /* Wrapped: copy tail segment first, then the head segment */
        append_resp(&s_dma_buf[s_dma_prev], SIM7600_DMA_BUF_SIZE - s_dma_prev);
        append_resp(&s_dma_buf[0], new_head);
    }

    s_dma_prev = new_head;
}

/* ── Public API ────────────────────────────────────────────────────────────── */

/**
 * @brief  Pulse PWRKEY via ARD_D9 (PH13) to power on the SIM7600G-H.
 */
void SIM7600_PowerOn(void)
{
    printf("[SIM] Powering on modem...\n");

    /* Assert PWRKEY: D9 HIGH drives the NPN transistor, pulling PWRKEY LOW */
    HAL_GPIO_WritePin(SIM7600_PWRKEY_PORT, SIM7600_PWRKEY_PIN, GPIO_PIN_SET);
    HAL_Delay(500u);

    /* Release PWRKEY */
    HAL_GPIO_WritePin(SIM7600_PWRKEY_PORT, SIM7600_PWRKEY_PIN, GPIO_PIN_RESET);

    /* Wait for the module to boot — SIM7600G-H takes up to 8 s.
       The modem will emit unsolicited "RDY" when ready, but we simply
       wait and let SIM7600_TestAT() confirm it's alive. */
    printf("[SIM] Waiting for modem boot (8 s)...\n");
    HAL_Delay(8000u);
}

/**
 * @brief  Initialise driver and start circular DMA + IDLE detection.
 */
void SIM7600_Init(UART_HandleTypeDef *huart)
{
    s_huart    = huart;
    s_resp_len = 0;
    s_dma_prev = 0;
    memset(s_dma_buf,  0, sizeof(s_dma_buf));
    memset(s_resp_buf, 0, sizeof(s_resp_buf));

    /* Start continuous receive: DMA fills s_dma_buf in a circle,
       IDLE line interrupt tells us when the modem paused sending. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, SIM7600_DMA_BUF_SIZE);

    /* Disable half-transfer interrupt — we only need IDLE + full-transfer */
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief  Retry "AT" ping until the modem responds or total_ms elapses.
 *         Mirrors TinyGSM's testAT(): 200 ms window per attempt, 100 ms gap.
 */
SIM7600_Status SIM7600_TestAT(uint32_t total_ms)
{
    printf("[SIM] Testing comms (up to %lums)...\n", total_ms);
    uint32_t t_start = HAL_GetTick();

    while ((HAL_GetTick() - t_start) < total_ms)
    {
        if (SIM7600_SendAT("", "OK", NULL, 0, 200u) == SIM7600_OK)
        {
            printf("[SIM] Modem responded OK\n");
            return SIM7600_OK;
        }
        HAL_Delay(100u);
    }

    printf("[SIM] Modem not responding after %lums\n", total_ms);
    return SIM7600_TIMEOUT;
}

/**
 * @brief  Send an AT command and block until the expected response arrives
 *         or the timeout expires.
 */
SIM7600_Status SIM7600_SendAT(const char *cmd,
                               const char *expected,
                               char       *resp_buf,
                               uint16_t    resp_buf_len,
                               uint32_t    timeout_ms)
{
    /* ── Build TX string ── */
    char tx_buf[128];
    if (cmd != NULL && cmd[0] != '\0')
        snprintf(tx_buf, sizeof(tx_buf), "AT%s\r\n", cmd);
    else
        snprintf(tx_buf, sizeof(tx_buf), "AT\r\n");

    /* ── Flush RX state, sync tail pointer to current DMA position ──
     *   Reading the DMA counter gives remaining transfers; subtracting from
     *   the buffer size gives the current write head.                        */
    s_resp_len = 0;
    s_resp_buf[0] = '\0';
    uint16_t cur_pos = (uint16_t)(SIM7600_DMA_BUF_SIZE -
                        __HAL_DMA_GET_COUNTER(s_huart->hdmarx));
    s_dma_prev = cur_pos % SIM7600_DMA_BUF_SIZE;

    /* ── Transmit ── */
    printf("[SIM] >> %.*s\n", (int)(strlen(tx_buf) - 2), tx_buf); /* strip \r\n */
    HAL_UART_Transmit(s_huart, (uint8_t *)tx_buf, (uint16_t)strlen(tx_buf), 1000u);

    /* ── Wait for expected response ── */
    uint32_t t_start = HAL_GetTick();

    while ((HAL_GetTick() - t_start) < timeout_ms)
    {
        if (s_resp_len > 0u)
        {
            if (strstr((char *)s_resp_buf, expected) != NULL)
            {
                if (resp_buf != NULL && resp_buf_len > 0u)
                {
                    strncpy(resp_buf, (char *)s_resp_buf, resp_buf_len - 1u);
                    resp_buf[resp_buf_len - 1u] = '\0';
                }
                printf("[SIM] << %s\n", (char *)s_resp_buf);
                return SIM7600_OK;
            }

            /* Modem replied with a hard error — no point waiting longer */
            if (strstr((char *)s_resp_buf, "\r\nERROR\r\n") != NULL ||
                strstr((char *)s_resp_buf, "+CME ERROR")   != NULL ||
                strstr((char *)s_resp_buf, "+CMS ERROR")   != NULL)
            {
                if (resp_buf != NULL && resp_buf_len > 0u)
                {
                    strncpy(resp_buf, (char *)s_resp_buf, resp_buf_len - 1u);
                    resp_buf[resp_buf_len - 1u] = '\0';
                }
                printf("[SIM] << %s\n", (char *)s_resp_buf);
                return SIM7600_ERROR;
            }
        }

        HAL_Delay(1u);
    }

    printf("[SIM] TIMEOUT waiting for \"%s\"\n", expected);
    return SIM7600_TIMEOUT;
}

/**
 * @brief  Called from the HAL_UARTEx_RxEventCallback() override in main.c.
 *         Drains new bytes from the circular DMA buffer.
 *
 *         Size semantics from HAL:
 *           IDLE event → bytes written so far in this DMA cycle (1 .. N-1)
 *           TC   event → N  (buffer fully filled; head wraps to 0)
 *           HT   event → N/2 (disabled, but handled for safety)
 */
void SIM7600_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance)
        return;

    /* Convert absolute byte-count to a 0-based buffer index.
       When size == SIM7600_DMA_BUF_SIZE (TC), new_head = 0 (full wrap). */
    uint16_t new_head = size % SIM7600_DMA_BUF_SIZE;

    drain_dma(new_head);
}
