/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sim7600.c
  * @brief   SIM7600G AT command driver — LPUART1 + circular DMA RX
  *
  * Design notes
  * ────────────
  * HAL_UARTEx_ReceiveToIdle_DMA() is used to run a continuous circular DMA
  * receive.  The DMA fires three kinds of events:
  *
  *   • IDLE  – modem stopped sending (fires HAL_UARTEx_RxEventCallback with
  *             Size = bytes written so far in this DMA cycle)
  *   • HT    – half-transfer (disabled in SIM7600_Init to reduce noise)
  *   • TC    – full buffer filled; Size == SIM7600_DMA_BUF_SIZE, DMA head
  *             wraps back to 0
  *
  * A tail-chase pointer (s_dma_prev) tracks where we last read from the
  * circular DMA buffer.  Each callback drains the new bytes into s_resp_buf
  * which SendAT then searches for the expected token.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sim7600.h"
#include <string.h>
#include <stdio.h>

/* ── Private state ─────────────────────────────────────────────────────────── */

static UART_HandleTypeDef *s_huart = NULL;

/* Raw circular DMA destination — written by DMA, read by drain_dma() */
static uint8_t  s_dma_buf[SIM7600_DMA_BUF_SIZE];

/* Accumulated modem response for the current SendAT call */
static uint8_t  s_resp_buf[SIM7600_RESP_BUF_SIZE];
static uint16_t s_resp_len = 0;

/* Last position we drained up to in s_dma_buf (tail pointer) */
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

SIM7600_Status SIM7600_SendSMS(const char *number, const char *sms_body)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "+CMGS=\"%s\"", number);

    // Step 1: Send +CMGS and wait for '>'
    SIM7600_Status status = SIM7600_SendAT(cmd, ">", NULL, 0, SIM7600_TIMEOUT_MEDIUM);
    if (status != SIM7600_OK) return status;

    // Step 2: Send actual message text (raw, no AT prepended)
    HAL_UART_Transmit(s_huart, (uint8_t*)sms_body, strlen(sms_body), 1000);

    // Step 3: Send Ctrl-Z to finalize
    uint8_t ctrl_z = 0x1A;
    HAL_UART_Transmit(s_huart, &ctrl_z, 1, 1000);

    // Step 4: Wait for +CMGS: and OK
    uint32_t t_start = HAL_GetTick();
    while ((HAL_GetTick() - t_start) < 20000) // 20s timeout
    {
        if (s_resp_len > 0)
        {
            if (strstr((char*)s_resp_buf, "+CMGS:") && strstr((char*)s_resp_buf, "OK"))
            {
                printf("[SIM] SMS Sent Successfully: %s\n", s_resp_buf);
                return SIM7600_OK;
            }
            if (strstr((char*)s_resp_buf, "ERROR"))
            {
                printf("[SIM] SMS Failed: %s\n", s_resp_buf);
                return SIM7600_ERROR;
            }
        }
        HAL_Delay(1);
    }

    printf("[SIM] SMS Send Timeout\n");
    return SIM7600_TIMEOUT;
}
#define HTTP_MAX_RESP 512

SIM7600_Status SIM7600_HTTPPost1(const char *url,
                                const char *payload,
                                char *resp_buf,
                                uint16_t resp_buf_len)
{
    char at_resp[256];
    SIM7600_Status status;
    int data_len = 0;

    if (!url || !payload || !resp_buf || resp_buf_len == 0)
        return SIM7600_ERROR;

    // 1. Set HTTP SSL mode off (plain HTTP; set 1 for HTTPS)
    status = SIM7600_SendAT("+HTTPSSL=0", "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) return status;

    // 2. Initialize HTTP service
    status = SIM7600_SendAT("+HTTPINIT", "OK", at_resp, sizeof(at_resp), 5000);
    if (status != SIM7600_OK) return status;

    // 3. Set PDP context ID
    status = SIM7600_SendAT("+HTTPPARA=\"CID\",1", "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 4. Set URL
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "+HTTPPARA=\"URL\",\"%s\"", url);
    status = SIM7600_SendAT(cmd, "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 5. Set content type
    status = SIM7600_SendAT("+HTTPPARA=\"CONTENT\",\"text/plain\"", "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 6. Send payload
    snprintf(cmd, sizeof(cmd), "+HTTPDATA=%d,10000", (int)strlen(payload));
    status = SIM7600_SendAT(cmd, "DOWNLOAD", at_resp, sizeof(at_resp), 5000);
    if (status != SIM7600_OK) goto cleanup;

    // Transmit payload
    HAL_UART_Transmit(s_huart, (uint8_t*)payload, (uint16_t)strlen(payload), 5000);

    // 7. Execute POST
    status = SIM7600_SendAT("+HTTPACTION=1", "+HTTPACTION:", at_resp, sizeof(at_resp), 15000);
    if (status != SIM7600_OK) goto cleanup;

    // Parse data length from +HTTPACTION response
    int status_code = 0;
    if (sscanf(at_resp, "+HTTPACTION:%*d,%d,%d", &status_code, &data_len) != 2)
    {
        printf("[SIM] Failed to parse +HTTPACTION response: %s\n", at_resp);
        status = SIM7600_ERROR;
        goto cleanup;
    }
    printf("[SIM] HTTP POST returned status %d, data length %d\n", status_code, data_len);

    // 8. Read HTTP response
    if (data_len > 0)
    {
        if ((uint16_t)data_len > resp_buf_len - 1) data_len = resp_buf_len - 1; // truncate if too long
        snprintf(cmd, sizeof(cmd), "+HTTPREAD=0,%d", data_len);
        status = SIM7600_SendAT(cmd, "OK", resp_buf, resp_buf_len, 10000);
        if (status != SIM7600_OK) goto cleanup;
    }
    else
    {
        resp_buf[0] = '\0'; // no response body
    }

cleanup:
    SIM7600_SendAT("+HTTPTERM", "OK", at_resp, sizeof(at_resp), 5000);
    return status;
}

SIM7600_Status SIM7600_HTTPPost(const char *url, const char *payload, char *resp_buf, uint16_t resp_buf_len)
{
    char at_resp[256];
    SIM7600_Status status;

    // 1. Initialize HTTP service
    status = SIM7600_SendAT("+HTTPINIT", "OK", at_resp, sizeof(at_resp), 5000);
    if (status != SIM7600_OK) return status;

    // 2. Set PDP context ID (usually 1)
    status = SIM7600_SendAT("+HTTPPARA=\"CID\",1", "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 3. Set target URL
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "+HTTPPARA=\"URL\",\"%s\"", url);
    status = SIM7600_SendAT(cmd, "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 4. Set content type
    snprintf(cmd, sizeof(cmd), "+HTTPPARA=\"CONTENT\",\"text/plain\"");
    status = SIM7600_SendAT(cmd, "OK", at_resp, sizeof(at_resp), 2000);
    if (status != SIM7600_OK) goto cleanup;

    // 5. Send data
    snprintf(cmd, sizeof(cmd), "+HTTPDATA=%d,10000", (int)strlen(payload)); // 10s timeout
    status = SIM7600_SendAT(cmd, "DOWNLOAD", at_resp, sizeof(at_resp), 5000);
    if (status != SIM7600_OK) goto cleanup;

    // 5b. Transmit payload
    HAL_UART_Transmit(s_huart, (uint8_t*)payload, (uint16_t)strlen(payload), 5000);

    // 6. Execute HTTP POST (1 = POST)
    status = SIM7600_SendAT("+HTTPACTION=1", "+HTTPACTION:", at_resp, sizeof(at_resp), 15000);
    if (status != SIM7600_OK) goto cleanup;

    // 6b. Parse HTTPACTION response for status code and data length
    int method, status_code, data_len;
    if (sscanf(at_resp, "+HTTPACTION:%d,%d,%d", &method, &status_code, &data_len) != 3)
    {
        printf("[SIM] Failed to parse +HTTPACTION response: %s\n", at_resp);
        status = SIM7600_ERROR;
        goto cleanup;
    }

    printf("[SIM] HTTP POST status: %d, data length: %d\n", status_code, data_len);

    // 7. Read HTTP response body (can be partial)
    if (data_len > 0 && resp_buf != NULL && resp_buf_len > 0)
    {
        snprintf(cmd, sizeof(cmd), "+HTTPREAD=0,%d", data_len); // read entire response
        status = SIM7600_SendAT(cmd, "OK", resp_buf, resp_buf_len, 10000);
        if (status != SIM7600_OK) goto cleanup;
        printf("[SIM] HTTP response:\n%s\n", resp_buf);
    }

cleanup:
    // 8. Terminate HTTP service
    SIM7600_SendAT("+HTTPTERM", "OK", at_resp, sizeof(at_resp), 5000);
    return status;
}
SIM7600_Status SIM7600_SendRawHTTPPost(const char *host, uint16_t port, const char *payload)
{
    char resp[256];
    char cmd[128];
    SIM7600_Status status;

    // 1. Open TCP socket on channel 0 (SIM7600 uses CIPOPEN, not CIPSTART)
    //    Response is async: "OK" arrives first, then "+CIPOPEN: 0,0" (0 = success)
    snprintf(cmd, sizeof(cmd), "+CIPOPEN=0,\"TCP\",\"%s\",%d", host, port);
    status = SIM7600_SendAT(cmd, "+CIPOPEN: 0,0", resp, sizeof(resp), SIM7600_TIMEOUT_LONG);
    if (status != SIM7600_OK) return status;

    printf("[SIM] Connected to %s:%d\n", host, port);

    // 2. Prepare raw HTTP POST
    //    Connection: close tells the server to close after responding so we know
    //    when the response ends — required for HTTP/1.1 (keep-alive by default)
    char http_request[512];
    snprintf(http_request, sizeof(http_request),
             "POST /post HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: text/plain\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             host, (int)strlen(payload), payload);

    // 3. Send HTTP length — channel 0 required on SIM7600
    snprintf(cmd, sizeof(cmd), "+CIPSEND=0,%d", (int)strlen(http_request));
    status = SIM7600_SendAT(cmd, ">", resp, sizeof(resp), SIM7600_TIMEOUT_MEDIUM);
    if (status != SIM7600_OK) return status;

    // 4. Send raw HTTP data directly — SIM7600_SendAT cannot be used here because
    //    it prepends "AT" and truncates to 128 bytes. After the '>' prompt the modem
    //    expects raw bytes followed by SEND OK, same pattern as SMS Ctrl-Z.
    //
    // Atomic flush: disable interrupts so ISR cannot fire between clearing s_resp_len
    // and s_resp_buf[0] — a GPS URC or other async byte arriving in that window would
    // leave s_resp_buf[0]=='\0' with s_resp_len>0, making strstr blind to later data.
    __disable_irq();
    s_resp_len = 0;
    s_resp_buf[0] = '\0';
    uint16_t cur_pos2 = (uint16_t)(SIM7600_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));
    s_dma_prev = cur_pos2 % SIM7600_DMA_BUF_SIZE;
    __enable_irq();

    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(s_huart, (uint8_t*)http_request, (uint16_t)strlen(http_request), 5000u);
    if (tx_status != HAL_OK)
    {
        printf("[SIM] HAL_UART_Transmit failed: %d\n", (int)tx_status);
        return SIM7600_ERROR;
    }

    // SIM7600 confirms TCP send with "+CIPSEND: <conn>,<sent>,<acked>" — NOT "SEND OK".
    // With Connection: close, the server response and "+IPCLOSE: 0,1" often arrive in
    // the same DMA burst as the +CIPSEND confirm, so we handle it all in one loop.
    status = SIM7600_TIMEOUT;
    uint32_t t_send = HAL_GetTick();
    while ((HAL_GetTick() - t_send) < SIM7600_TIMEOUT_LONG)
    {
        if (strstr((char*)s_resp_buf, "+CIPSEND:"))  { status = SIM7600_OK;    break; }
        if (strstr((char*)s_resp_buf, "SEND FAIL"))  { status = SIM7600_ERROR; break; }
        if (strstr((char*)s_resp_buf, "+IPCLOSE: 0,1"))
        {
            // Remote closed before we saw +CIPSEND — data was still sent; treat as OK.
            status = SIM7600_OK;
            break;
        }
        if (strstr((char*)s_resp_buf, "ERROR"))      { status = SIM7600_ERROR; break; }
        HAL_Delay(1u);
    }
    if (status != SIM7600_OK)
    {
        printf("[SIM] CIPSEND failed (status=%d), modem said: %s\n", status, (char*)s_resp_buf);
        return status;
    }

    // 4b. Wait for server response and connection close.
    //     Do NOT flush here — the HTTP response often arrives in the same DMA burst
    //     as +CIPSEND and is already partially in the DMA buffer. Flushing would
    //     advance s_dma_prev past the response header, dropping the start of the reply.
    //     With Connection: close the server closes after replying → "+IPCLOSE: 0,1".
    //     Use "HTTP/1.1 2" (includes the status digit) to avoid matching the echoed
    //     request line "POST /post HTTP/1.1" which has no status digit after the version.
    uint32_t t_rx = HAL_GetTick();
    while ((HAL_GetTick() - t_rx) < SIM7600_TIMEOUT_LONG)
    {
        if (strstr((char*)s_resp_buf, "+IPCLOSE: 0,"))
            break;
        HAL_Delay(1u);
    }
    printf("[SIM] HTTP response: %s\n", (char*)s_resp_buf);

    if (strstr((char*)s_resp_buf, "HTTP/1.1 2") != NULL)
        printf("[SIM] HTTP success (2xx)\n");
    else if (strstr((char*)s_resp_buf, "HTTP/1.1") != NULL)
        printf("[SIM] HTTP error — check status line above\n");
    else
        printf("[SIM] HTTP response not captured\n");

    // 5. Connection: close means the server already closed it (+IPCLOSE: 0,1).
    //    Only send CIPCLOSE if still open.
    if (strstr((char*)s_resp_buf, "+IPCLOSE:") == NULL)
    {
        status = SIM7600_SendAT("+CIPCLOSE=0", "+CIPCLOSE: 0,0", resp, sizeof(resp), SIM7600_TIMEOUT_SHORT);
        if (status != SIM7600_OK) return status;
    }

    printf("[SIM] TCP socket closed\n");
    return SIM7600_OK;
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
