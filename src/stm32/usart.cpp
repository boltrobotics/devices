// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#if defined(BTR_ENABLE_USART)

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usart.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// PROJECT INCLUDES
#include "devices/stm32/usart.hpp"  // class implemented

#if !defined(TX_Q_SIZE)
#define TX_Q_SIZE 64
#endif
#if !defined(RX_Q_SIZE)
#define RX_Q_SIZE 64
#endif
#if !defined(RX_BUFF_SIZE)
#define RX_BUFF_SIZE 64
#endif
#if !defined(TX_BUFF_SIZE)
#define TX_BUFF_SIZE 64
#endif
#if !defined(TX_Q_DELAY)
#define TX_Q_DELAY 10
#endif

extern "C" {

#if 0
////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt handling {

static void onInterrupt(uint8_t usart_id)
{
	struct s_uart* uartp = uart_data[usart_id];

	if (!uartp) {
    // Not open for ISR receiving
		return;
  }

	uint32_t uart = uarts[usart_id].usart;

	while (USART_SR(uart) & USART_SR_RXNE) {
    // Read data
		char ch = USART_DR(uart);
    // Calc next tail index
		uint32_t ntail = (uartp->tail + 1) % USART_BUF_DEPTH;

		// Save data if the buffer is not full.
    //
		if (ntail != uartp->head) {
			uartp->buf[uartp->tail] = ch;
			uartp->tail = ntail;
		}
	}
}

void usart1_isr()
{
	onInterrupt(0);
}
void usart2_isr()
{
	onInterrupt(1);
}
void usart3_isr()
{
	onInterrupt(3);
}

// } Interrupt handling
////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

static void onDataRecv(usbd_device* usbd_dev, uint8_t ep)
{
  (void) ep;

  uint32_t rx_q_avail = uxQueueSpacesAvailable(rx_q_);

  if (rx_q_avail > 0) {
    char buff[RX_BUFF_SIZE];

    int bytes = (RX_BUFF_SIZE < rx_q_avail ? RX_BUFF_SIZE : rx_q_avail);
    bytes = usbd_ep_read_packet(usbd_dev, 0x01, buff, bytes);

    for (int i = 0; i < bytes; ++i) {
      xQueueSend(rx_q_, &buff[i], 0);
    }
  }
}

static void txTask(void* arg)
{
  Usart* dev = (Usart*) arg;
  char ch;

  for (;;) {
		if (pdPASS == xQueueReceive(dev->tx_q_, &ch, 500)) {
			while (false == usart_get_flag(dev->id_, USART_SR_TXE)) {
				taskYIELD();
      }
			usart_send(dev->id_, ch);
    }
  }
}

static int initUsart(
    Usart* dev,
    uint32_t id,
    uint32_t rcc_clk,
    uint8_t irqn,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t stop_bits,
    int parity,
    int flow_ctrl,
    int io_mode,
    const char* tx_name,
    const char* rx_name,
    TaskHandle_t* tx_h,
    TaskHandle_t* rx_h,
    QueueHandle_t* tx_q,
    QueueHandle_t* rx_q,
    uint32_t tx_q_size,
    uint32_t rx_q_size)
{
  rcc_periph_clock_enable(rcc_clk);
  usart_set_baudrate(id, baud_rate);
  usart_set_databits(id, data_bits);
  usart_set_stopbits(id, stop_bits);
  usart_set_parity(id, parity);
  usart_set_flow_control(id, flow_ctrl);
  usart_set_mode(id, io_mode);

  if (NULL == (tx_q = xQueueCreate(tx_q_size, sizeof(char)))) {
    goto cleanup;
  } else if (NULL == (rx_q = xQueueCreate(rx_q_size, sizeof(char)))) {
    goto cleanup;
  }

  uint16_t stack_size = configMINIMAL_STACK_SIZE;

  if (pdPASS != xTaskCreate(txTask, tx_name, stack_size, dev, priority, tx_h)) {
    goto cleanup;
  } else if (pdPASS != xTaskCreate(rxTask, rx_name, stack_size, dev, priority, rx_h)) {
    goto cleanup;
  }

  cm_disable_interrupts();
  nvic_enable_irq(irqn);
  usart_enable_rx_interrupt(id);
  usart_enable(id);
  cm_enable_interrupts();
	return 0;

  cleanup:
    rcc_periph_clock_disable(rcc_clk);
    if (NULL != rx_h) { vTaskDelete(rx_h); rx_h = NULL; }
    if (NULL != tx_h) { vTaskDelete(tx_h); tx_h = NULL; }
    if (NULL != rx_q) { vQueueDelete(rx_q); rx_q = NULL; }
    if (NULL != tx_q) { vQueueDelete(tx_q); tx_q = NULL; }
    return -1;
}

} // extern "C"

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

// static
Usart* Usart::instance(uint32_t usart_id)
{
  switch (usart_id) {
    case 1: {
#if defined(BTR_USART1_ENABLED)
      static Usart usart_1;

      if (false == usart_1->isOpen()) {
        initUsart(...);
        //RCC_USART1, NVIC_USART1_IRQ
      }
      return &usart_1;
#else
      return NULL;
#endif
    }
    case 2: {
#if defined(BTR_ENABLE_USART2)
      static Usart usart_2(usart_id, baud_rate, data_bits, parity);
      //RCC_USART2, NVIC_USART2_IRQ
      return &usart_2;
#else
      return NULL;
#endif
    }
    case 3: {
#if defined(BTR_ENABLE_USART3)
      static Usart usart_3(usart_id, baud_rate, data_bits, parity);
      //RCC_USART3, NVIC_USART3_IRQ
      return &usart_3;
#else
      return NULL;
#endif
    }
    default:
      return NULL;
  }
}

bool Usart::isOpen() const
{
  return (tx_q_ != NULL || rx_q_ != NULL);
}

int Usart::open(bool init_gpio, uint32_t priority)
{
  if (false == ready_) {
    initUsb(init_gpio, priority);

    while (false == ready_) {
      taskYIELD();
    }
  }
  return 0;
}

void Usart::close()
{
  // TODO turn off the clocks, stop transmit task, etc.
}

int Usart::setTimeout(uint32_t timeout)
{
  (void) timeout;
}

int Usart::available()
{
  return uxQueueMessagesWaiting(rx_q_);
}

int Usart::send(char ch)
{
  return (pdPASS == xQueueSend(tx_q_, &ch, TX_Q_DELAY) ? 0 : -1);
}

int Usart::send(const char* buff)
{
  int rc = 0;

  while (*buff) {
    if (0 != (rc = send(*buff++))) {
      break;
    }
  }
  return rc;
}

int Usart::send(const char* buff, uint16_t bytes)
{
  int rc = 0;

  while (bytes-- > 0) {
    if (0 != (rc = send(*buff))) {
      break;
    }
    ++buff;
  }
  return rc;
}

int Usart::recv()
{
  char ch;

  if (xQueueReceive(rx_q_, &ch, TX_Q_DELAY) != pdPASS) {
    return -1;
  }
  return ch;
}

int Usart::recv(char* buff, uint16_t bytes)
{
  int32_t byte_idx = 0;

  while (bytes > 0 && (byte_idx + 1) < bytes) {
    int ch = recv();
    buff[byte_idx++] = (char) ch;
  }

  buff[byte_idx] = 0;
  return byte_idx;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart(const char* name)
  :
    HardwareStream()
    open_(false),
    name_(name),
    tx_q_(NULL),
    rx_q_(NULL)
{
}

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // defined(BTR_ENABLE_USART)
