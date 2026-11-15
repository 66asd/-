#include "hmi_uart.h"
#include "usart.h"
static HmiProtocol protocol;
static uint8_t rx_byte, rx_ring[128];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t port_error;

static int send(void *context, const uint8_t *data, uint16_t size)
{
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)context;
    return HAL_UART_Transmit_IT(uart, (uint8_t *)data, size) == HAL_OK;
}
int HmiUart_Init(void)
{
    rx_head = rx_tail = 0u;
    port_error = 0u;
    HmiProtocol_Init(&protocol, send, &huart1, HAL_GetTick());
    return HAL_UART_Receive_IT(&huart1, &rx_byte, 1u) == HAL_OK;
}
HmiProtocol *HmiUart_Protocol(void) { return &protocol; }
void HmiUart_Process(void)
{
    if (protocol.transmitting && (uint32_t)(HAL_GetTick()-protocol.tx_started)>200u) port_error=1u;
    if (port_error) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        port_error = 0u;
        rx_tail = rx_head;
        __set_PRIMASK(mask);
        (void)HAL_UART_Abort(&huart1);
        HmiProtocol_TransportError(&protocol);
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1u) != HAL_OK) port_error = 1u;
    }
    while (rx_tail != rx_head) {
        uint8_t byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) & 127u);
        HmiProtocol_Receive(&protocol, byte);
    }
    HmiProtocol_Process(&protocol, HAL_GetTick());
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) {
        uint16_t next = (uint16_t)((rx_head + 1u) & 127u);
        if (next == rx_tail) port_error = 1u;
        else { rx_ring[rx_head] = rx_byte; __DMB(); rx_head = next; }
        if (HAL_UART_Receive_IT(uart, &rx_byte, 1u) != HAL_OK) port_error = 1u;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) HmiProtocol_TxComplete(&protocol);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) port_error = 1u;
}
