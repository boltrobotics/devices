// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "devices/usart.hpp"  // class implemented

#if BTR_USART0_ENABLED > 0 || BTR_USART1_ENABLED > 0 || BTR_USART2_ENABLED > 0

// Internal macros
#ifndef BTR_USART_PRIORITY
#define BTR_USART_PRIORITY (configMAX_PRIORITIES - 1)
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static {

#if BTR_USART0_ENABLED > 0
static btr::Usart usart_0(
  RCC_GPIOA, RCC_USART1, GPIOA, USART1, NVIC_USART1_IRQ, GPIO_USART1_TX, GPIO_USART1_RX,
  GPIO11, GPIO12);
#endif
#if BTR_USART1_ENABLED > 0
static btr::Usart usart_1(
  RCC_GPIOA, RCC_USART2, GPIOA, USART2, NVIC_USART2_IRQ, GPIO_USART2_TX, GPIO_USART2_RX,
  GPIO0, GPIO1);
#endif
#if BTR_USART2_ENABLED > 0
static btr::Usart usart_2(
  RCC_GPIOB, RCC_USART3, GPIOB, USART3, NVIC_USART3_IRQ, GPIO_USART3_TX, GPIO_USART3_RX,
  GPIO13, GPIO14);
#endif

extern "C" {

static void txTask(void* arg)
{
  btr::Usart* u = (btr::Usart*) arg;
  char ch;

  for (;;) {
    if (pdPASS == xQueueReceive(u->tx_q_, &ch, 500)) {
      while (false == usart_get_flag(u->pin_, USART_SR_TXE)) {
        taskYIELD();
      }
      usart_send(u->pin_, ch);
      LED_TOGGLE();
    }
  }
}

static void onRecv(btr::Usart* u)
{
  while (USART_SR(u->pin_) & USART_SR_RXNE) {
    char ch = USART_DR(u->pin_);
    uint32_t tail_next = (u->rx_tail_ + 1) % BTR_USART_RX_BUFF_SIZE;

    // Save data if buffer has room, discard the data if there is no room.
    if (tail_next != u->rx_head_) {
      u->rx_buff_[u->rx_tail_] = ch;
      u->rx_tail_ = tail_next;
    }
  }
  LED_TOGGLE();
}

#if BTR_USART0_ENABLED > 0
void usart1_isr()
{
  onRecv(&usart_0);
}
#endif
#if BTR_USART1_ENABLED > 0
void usart2_isr()
{
  onRecv(&usart_1);
}
#endif
#if BTR_USART2_ENABLED > 0
void usart3_isr()
{
  onRecv(&usart_2);
}
#endif

} // extern "C"

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart(
    rcc_periph_clken rcc_gpio,
    rcc_periph_clken rcc_usart,
    uint32_t port,
    uint32_t pin,
    uint32_t irq,
    uint16_t tx,
    uint16_t rx,
    uint16_t cts,
    uint16_t rts)
  :
    rcc_gpio_(rcc_gpio),
    rcc_usart_(rcc_usart),
    port_(port),
    pin_(pin),
    irq_(irq),
    tx_(tx),
    rx_(rx),
    cts_(cts),
    rts_(rts),
    tx_q_(nullptr),
    rx_error_(0),
    enable_flush_(false),
    rx_head_(0),
    rx_tail_(0),
    rx_buff_()
{
} 

//============================================= OPERATIONS =========================================

// static
Usart* Usart::instance(uint32_t id, bool open)
{
  switch (id) {
#if BTR_USART0_ENABLED > 0
    case 0:
      if (open) {
        usart_0.open(BTR_USART0_BAUD, BTR_USART0_DATA_BITS, BTR_USART0_STOP_BITS, BTR_USART0_PARITY);
      }
      return &usart_0;
#endif
#if BTR_USART1_ENABLED > 0
    case 1:
      if (open) {
        usart_1.open(BTR_USART1_BAUD, BTR_USART1_DATA_BITS, BTR_USART1_STOP_BITS, BTR_USART1_PARITY);
      }
      return &usart_1;
#endif
#if BTR_USART2_ENABLED > 0
    case 2:
      if (open) {
        usart_2.open(BTR_USART2_BAUD, BTR_USART2_DATA_BITS, BTR_USART2_STOP_BITS, BTR_USART2_PARITY);
      }
      return &usart_2;
#endif
    default:
      return nullptr;
  }
}

bool Usart::isOpen()
{
  return enable_flush_;
}

int Usart::open(
    uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity, const char* port)
{
  (void) port; // not used on STM32

  if (true == isOpen()) {
    return 0;
  }

  usart_disable_rx_interrupt(pin_);

  int flow_ctrl = USART_FLOWCONTROL_NONE;

  if (rts_) {
    if (cts_) {
      flow_ctrl = USART_FLOWCONTROL_RTS_CTS;
      gpio_set_mode(port_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, cts_); 
      gpio_set_mode(port_, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rts_);
    } else {
      flow_ctrl = USART_FLOWCONTROL_RTS;
      gpio_set_mode(port_, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rts_);
    }
  } else if (cts_) {
    flow_ctrl = USART_FLOWCONTROL_CTS;
    gpio_set_mode(port_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, cts_); 
  }

  rcc_periph_clock_enable(rcc_gpio_);
  rcc_periph_clock_enable(rcc_usart_);
  gpio_set_mode(port_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, tx_);
  gpio_set_mode(port_, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rx_);

  usart_set_baudrate(pin_, baud);
  usart_set_databits(pin_, data_bits);
  usart_set_mode(pin_, USART_MODE_TX_RX);
  usart_set_flow_control(pin_, flow_ctrl);

  switch (parity) {
    case EVEN:
      usart_set_parity(pin_, USART_PARITY_EVEN);
      break;
    case ODD:
      usart_set_parity(pin_, USART_PARITY_ODD);
      break;
    case NONE:
    default:
      usart_set_parity(pin_, USART_PARITY_NONE);
  }

  switch (stop_bits) {
    case ONEPOINTFIVE:
      usart_set_stopbits(pin_, USART_STOPBITS_1_5);
      break;
    case TWO:
      usart_set_stopbits(pin_, USART_STOPBITS_2);
      break;
    case ONE:
    default:
      usart_set_stopbits(pin_, USART_STOPBITS_1);
  }

  if (nullptr == (tx_q_ = xQueueCreate(BTR_USART_TX_BUFF_SIZE, sizeof(char)))) {
    goto cleanup;
  }

  if (pdPASS != xTaskCreate(
        txTask, nullptr, configMINIMAL_STACK_SIZE, this, BTR_USART_PRIORITY, nullptr)) {
    goto cleanup;
  }

  nvic_enable_irq(irq_);
  usart_enable_rx_interrupt(pin_);
  usart_enable(pin_);
  enable_flush_ = true;
  return 0;

  cleanup:
    close();
    return -1;
}

void Usart::close()
{
  enable_flush_ = false;
  usart_disable_rx_interrupt(pin_);
  nvic_disable_irq(irq_);
  usart_disable(pin_);
  rcc_periph_clock_disable(rcc_usart_);
  rcc_periph_clock_disable(rcc_gpio_);

  if (nullptr != tx_q_) {
    vQueueDelete(tx_q_);
    tx_q_ = nullptr;
  }
}

int Usart::available()
{
  uint16_t bytes = BTR_USART_RX_BUFF_SIZE + rx_head_ - rx_tail_;
  return (bytes % BTR_USART_RX_BUFF_SIZE);
}

int Usart::flush(DirectionType dir)
{
  int rc = 0;

  if (dir == DirectionType::OUT || dir == DirectionType::INOUT) {
    if ((rc = uxQueueMessagesWaiting(tx_q_)) > 0) {
      vTaskDelay(pdMS_TO_TICKS(BTR_USART_TX_DELAY_MS));
      rc = uxQueueMessagesWaiting(tx_q_);
    }
  }
  return rc;
}

uint32_t Usart::send(const char* buff, uint16_t bytes, uint32_t timeout)
{
  uint32_t rc = 0;

  while (bytes > 0) {
    if (pdPASS != xQueueSend(tx_q_, buff, pdMS_TO_TICKS(timeout))) {
      rc |= BTR_USART_TIMEDOUT_ERR;
      break;
    }
    ++buff;
    ++rc;
    --bytes;
  }
  return rc;
}

uint32_t Usart::recv(char* buff, uint16_t bytes, uint32_t timeout)
{
  uint32_t rc = 0;
  uint32_t delay = 0;

  while (bytes > 0) {
    if (rx_head_ != rx_tail_) {
      *buff++ = rx_buff_[rx_tail_];  
      rx_tail_ = (rx_tail_ + 1 ) % BTR_USART_RX_BUFF_SIZE;
      delay = 0;
      --bytes;
      ++rc;
    } else {
      if (timeout > 0) {
        if (delay >= timeout) {
          rc |= BTR_USART_TIMEDOUT_ERR;
          break;
        } else {
          // Wait and test queue one time after the delay.
          vTaskDelay(pdMS_TO_TICKS(timeout));
          delay = timeout;
        }
      } else {
        taskYIELD();
      }
    }
  }

  rc |= (uint32_t(rx_error_) << 16);
  rx_error_ = 0;
  return rc;
}

} // namespace btr

#endif // BTR_USARTn_ENABLED > 0
