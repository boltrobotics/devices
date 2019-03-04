// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <errno.h>

// PROJECT INCLUDES
#include "devices/stm32/usart.hpp"  // class implemented

#if BTR_USART1_ENABLED > 0 || BTR_USART2_ENABLED > 0 || \
    BTR_USART3_ENABLED > 0 || BTR_USART4_ENABLED > 0

// Internal macros
#ifndef BTR_USART_PRIORITY
#define BTR_USART_PRIORITY (configMAX_PRIORITIES - 1)
#endif

extern "C" {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware I/O {

struct UsartInfo {
  rcc_periph_clken rcc_gpio;
  rcc_periph_clken rcc_usart;
  uint32_t port;
  uint32_t pin;
  uint32_t irq;
  uint16_t tx;
  uint16_t rx;
  uint16_t cts;
  uint16_t rts;
};

static struct UsartInfo usart_info_[] = {
  { RCC_GPIOA, RCC_USART1, GPIOA, USART1, NVIC_USART1_IRQ, GPIO_USART1_TX, GPIO_USART1_RX,
    GPIO11, GPIO12 },
  { RCC_GPIOA, RCC_USART2, GPIOA, USART2, NVIC_USART2_IRQ, GPIO_USART2_TX, GPIO_USART2_RX,
    GPIO0, GPIO1 },
  { RCC_GPIOB, RCC_USART3, GPIOB, USART3, NVIC_USART3_IRQ, GPIO_USART3_TX, GPIO_USART3_RX,
    GPIO13, GPIO14 }
};

static void txTask(void* arg)
{
  btr::Usart* dev = (btr::Usart*) arg;
  uint32_t pin = usart_info_[dev->id() - 1].pin;
  char ch;

  for (;;) {
    if (pdPASS == xQueueReceive(dev->txq(), &ch, 500)) {
      while (false == usart_get_flag(pin, USART_SR_TXE)) {
        taskYIELD();
      }
      LED_TOGGLE();
      usart_send(pin, ch);
    }
  }
}

static void onRecv(uint8_t id)
{
  btr::Usart* dev = btr::Usart::instance(id);

  if (NULL == dev) {
    return;
  }

  struct UsartInfo* info = &usart_info_[dev->id() - 1];

  while (USART_SR(info->pin) & USART_SR_RXNE) {
    char ch = USART_DR(info->pin);
    uint32_t ntail = (dev->rxTail() + 1) % BTR_USART_RX_BUFF_SIZE;

    // Save data if buffer has room, discard the data if there is no room.
    if (ntail != dev->rxHead()) {
      dev->rxBuff()[dev->rxTail()] = ch;
      dev->rxTail() = ntail;
    }
  }
  LED_TOGGLE();
}

void usart1_isr()
{
  onRecv(1);
}

void usart2_isr()
{
  onRecv(2);
}

void usart3_isr()
{
  onRecv(3);
}

// } Hardware I/O

static int initUsart(
    btr::Usart* dev,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t stop_bits,
    int parity,
    int rts,
    int cts,
    const char* tx_name,
    uint32_t tx_q_size,
    int priority)
{
  if (dev->id() < BTR_USART_MIN_ID || dev->id() > BTR_USART_MAX_ID) {
    errno = EINVAL;
    return -1;
  }

  struct UsartInfo* info = &usart_info_[dev->id() - 1];
  usart_disable_rx_interrupt(info->pin);

  switch (parity) {
    case 'N':
      parity = USART_PARITY_NONE;
      break;
    case 'O':
      parity = USART_PARITY_ODD;
      break;
    case 'E':
      parity = USART_PARITY_EVEN;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  int flow_ctrl = USART_FLOWCONTROL_NONE;

  if (rts) {
    if (cts) {
      flow_ctrl = USART_FLOWCONTROL_RTS_CTS;
      gpio_set_mode(info->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, info->cts); 
      gpio_set_mode(info->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, info->rts);
    } else {
      flow_ctrl = USART_FLOWCONTROL_RTS;
      gpio_set_mode(info->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, info->rts);
    }
  } else if (cts) {
    flow_ctrl = USART_FLOWCONTROL_CTS;
    gpio_set_mode(info->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, info->cts); 
  }

  rcc_periph_clock_enable(info->rcc_gpio);
  rcc_periph_clock_enable(info->rcc_usart);
  gpio_set_mode(info->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, info->tx);
  gpio_set_mode(info->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, info->rx);

  usart_set_baudrate(info->pin, baud_rate);
  usart_set_databits(info->pin, data_bits);
  usart_set_stopbits(info->pin, stop_bits);
  usart_set_parity(info->pin, parity);
  usart_set_mode(info->pin, USART_MODE_TX_RX);
  usart_set_flow_control(info->pin, flow_ctrl);

  if (NULL == (dev->txq() = xQueueCreate(tx_q_size, sizeof(char)))) {
    goto cleanup;
  }

  if (pdPASS != xTaskCreate(
        txTask, tx_name, configMINIMAL_STACK_SIZE, dev, priority, &dev->txh()))
  {
    goto cleanup;
  }

  nvic_enable_irq(info->irq);
  usart_enable_rx_interrupt(info->pin);
  usart_enable(info->pin);

  return 0;

  cleanup:
    usart_disable_rx_interrupt(info->pin);
    nvic_disable_irq(info->irq);
    usart_disable(info->pin);
    rcc_periph_clock_disable(info->rcc_usart);
    rcc_periph_clock_disable(info->rcc_gpio);
    if (NULL != dev->txh()) { vTaskDelete(dev->txh()); dev->txh() = NULL; }
    if (NULL != dev->txq()) { vQueueDelete(dev->txq()); dev->txq() = NULL; }
    return -1;
}

static void yield()
{
  taskYIELD();
}

} // extern "C"

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart(uint8_t id)
  :
  id_(id),
  tx_h_(NULL),
  tx_q_(NULL),
  rx_head_(0),
  rx_tail_(0),
  rx_buff_()
{
} 

//============================================= OPERATIONS =========================================

#if BTR_USART1_ENABLED > 0
static Usart usart_1(1);
#endif
#if BTR_USART2_ENABLED > 0
static Usart usart_2(2);
#endif
#if BTR_USART3_ENABLED > 0
static Usart usart_3(3);
#endif

// static
Usart* Usart::instance(uint32_t usart_id)
{
  switch (usart_id) {
#if BTR_USART1_ENABLED > 0
    case 1: {
      return &usart_1;
    }
#endif
#if BTR_USART2_ENABLED > 0
    case 2: {
      return &usart_2;
    }
#endif
#if BTR_USART3_ENABLED > 0
    case 3: {
      return &usart_3;
    }
#endif
    default:
      return nullptr;
  }
}

bool Usart::isOpen()
{
  return (tx_h_ != NULL);
}

int Usart::open()
{
  int rc = 0;

  switch (id_) {
    case 1:
      if (false == isOpen()) {
        rc = initUsart(
            this,
            BTR_USART1_BAUD,
            BTR_USART1_DATA_BITS,
            BTR_USART1_STOP_BITS,
            BTR_USART1_PARITY,
            BTR_USART1_RTS,
            BTR_USART1_CTS,
            BTR_USART1_NAME,
            BTR_USART_TX_BUFF_SIZE,
            BTR_USART_PRIORITY);
      }
    case 2:
      if (false == isOpen()) {
        rc = initUsart(
            this,
            BTR_USART2_BAUD,
            BTR_USART2_DATA_BITS,
            BTR_USART2_STOP_BITS,
            BTR_USART2_PARITY,
            BTR_USART2_RTS,
            BTR_USART2_CTS,
            BTR_USART2_NAME,
            BTR_USART_TX_BUFF_SIZE,
            BTR_USART_PRIORITY);
      }
    case 3:
      if (false == isOpen()) {
        rc = initUsart(
            this,
            BTR_USART3_BAUD,
            BTR_USART3_DATA_BITS,
            BTR_USART3_STOP_BITS,
            BTR_USART3_PARITY,
            BTR_USART3_RTS,
            BTR_USART3_CTS,
            BTR_USART3_NAME,
            BTR_USART_TX_BUFF_SIZE,
            BTR_USART_PRIORITY);
      }
    default:
      rc = -1;
  }
  return rc;
}

void Usart::close()
{
  // TODO turn off the clocks, stop transmit task, etc.
}

int Usart::setTimeout(uint32_t timeout)
{
  (void) timeout;
  return 0;
}

int Usart::available()
{
  uint16_t bytes = BTR_USART_RX_BUFF_SIZE + rx_head_ - rx_tail_;
  return (bytes % BTR_USART_RX_BUFF_SIZE);
}

int Usart::flush(DirectionType queue_selector)
{
  (void) queue_selector;

#if 0
  // TODO implement flush
  if (true == drain) {
    while (uxQueueMessagesWaiting(txq()) > 0) {
      yield();
    }
    struct UsartInfo* info = &usart_info_[id_ - 1];
    usart_send_blocking(info->pin, ch);
    return 0;
  }
#endif
  return 0;
}

int Usart::send(char ch)
{
  return (pdPASS == xQueueSend(tx_q_, &ch, pdMS_TO_TICKS(BTR_USART_TX_DELAY_MS)) ? 0 : -1);
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

int Usart::send(const char* buff, uint32_t bytes)
{
  (void) drain;
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
  char ch = -1;

  if (rx_head_ != rx_tail_) {
    ch = rx_buff_[rx_head_];  
    rx_head_ = (rx_head_ + 1 ) % BTR_USART_RX_BUFF_SIZE;
  }
  return ch;
}

int Usart::recv(char* buff, uint32_t bytes)
{
  uint32_t byte_idx = 0;

  while (bytes > 0 && (byte_idx + 1) < bytes) {
    int ch = recv();

    if (-1 == ch) {
      yield();
      continue;
    }
    buff[byte_idx++] = (char) ch;
  }

  buff[byte_idx] = 0;
  return byte_idx;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_USARTx_ENABLED > 0
