// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_PwmMotor2Wire_hpp_
#define _btr_PwmMotor2Wire_hpp_

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define GEAR_PRESCALER    4     // F_CPU 72MHz / 4 = 18MHz
#define GEAR_PWM_PERIOD   400   // Private timer 9MHz (i.e. 18MHz / 2 center-align) / 22.5KHz = 400
#define SERVO_PRESCALER   72    // F_CPU 72MHz / 72 = 1MHz
#define SERVO_PWM_PERIOD  20000 // Private timer 1MHz / 50Hz = 20000

namespace btr
{

class PwmMotor2Wire
{
public:

  enum MotorType
  {
    GEAR,
    SERVO
  };

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param max_duty - in ticks between 0 - GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  PwmMotor2Wire(
      MotorType motor_type,
      rcc_periph_clken rcc_timer_clk,
      uint32_t timer,
      tim_oc_id timer_ocid_fw,
      tim_oc_id timer_ocid_bw,
      rcc_periph_clken rcc_pwm_clk_fw,
      uint32_t pwm_port_fw,
      uint16_t pwm_pin_fw,
      rcc_periph_clken rcc_pwm_clk_bw,
      uint32_t pwm_port_bw,
      uint16_t pwm_pin_bw,
      uint16_t max_duty);

// OPERATIONS

  /**
   * @param speed - pwm value between 0 and max_duty()
   * @param forward - 0 - reverse, 1 - forward
   */
  void setSpeed(uint16_t speed, uint8_t forward);

  /**
   * @return maximum duty in ticks between 0 and GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  uint16_t max_duty() const;

private:

// ATTRIBUTES

  uint32_t timer_;
  tim_oc_id timer_ocid_fw_;
  tim_oc_id timer_ocid_bw_;
  uint32_t pwm_port_fw_;
  uint16_t pwm_pin_fw_;
  uint32_t pwm_port_bw_;
  uint16_t pwm_pin_bw_;
  uint16_t max_duty_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

PwmMotor2Wire::PwmMotor2Wire(
      MotorType motor_type,
      rcc_periph_clken rcc_timer_clk,
      uint32_t timer,
      tim_oc_id timer_ocid_fw,
      tim_oc_id timer_ocid_bw,
      rcc_periph_clken rcc_pwm_clk_fw,
      uint32_t pwm_port_fw,
      uint16_t pwm_pin_fw,
      rcc_periph_clken rcc_pwm_clk_bw,
      uint32_t pwm_port_bw,
      uint16_t pwm_pin_bw,
      uint16_t max_duty
    ) :
  timer_(timer),
  timer_ocid_fw_(timer_ocid_fw),
  timer_ocid_bw_(timer_ocid_bw),
  pwm_port_fw_(pwm_port_fw),
  pwm_pin_fw_(pwm_pin_fw),
  pwm_port_bw_(pwm_port_bw),
  pwm_pin_bw_(pwm_pin_bw),
  max_duty_(max_duty)
{
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(rcc_timer_clk);
  rcc_periph_clock_enable(rcc_pwm_clk_fw);
  rcc_periph_clock_enable(rcc_pwm_clk_bw);

  //gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM1_REMAP_NO_REMAP);

  gpio_set_mode(pwm_port_fw_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pwm_pin_fw_);
  gpio_set_mode(pwm_port_bw_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pwm_pin_bw_);

  timer_disable_counter(timer_);
  //rcc_periph_reset_pulse(RST_TIM1); // replaces reset_timer(timer_)

  if (motor_type == GEAR) {
    // Set timer to center-aligned mode (Phase & Frequency Correct).
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_2, TIM_CR1_DIR_UP);

    // Prescalers: PLL (72e6) -> APBn (72e6 / 2 = 36e6) -> global timer (36e6 * 2 = 72e6) ->
    //  private timer (72e6 / 4 = 18e6) -> PWM center-aligned (18e6 / 2 = 9MHz)
    timer_set_prescaler(timer_, GEAR_PRESCALER);

    // Sets TIMx_ARR register (pwm period).
    // For servo at 50Hz: 180000 ticks at 9MHz => 20 mS period (180000 * 1/9e6)
    // For gear motor at 22,500Hz = 400 ticks at 9MHz clock => 44 uS period (400 * 1/9e6)
    // Only pulse width matters for servos (not PWM duty cycle, e.g. 20mS)
    timer_set_period(timer_, GEAR_PWM_PERIOD);
  } else {
    // To conserve power, stop sending servo control pulses (change mode to input, AVRP.220)
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_, SERVO_PRESCALER);
    timer_set_period(timer_, SERVO_PWM_PERIOD);
  }

  timer_enable_preload(timer_);
  timer_continuous_mode(timer_);

  // PWM1 mode sets output pin when TIMx_CNT < TIMx_CCR, otherwise it clears it.
  timer_disable_oc_output(timer_, timer_ocid_fw_);
  timer_disable_oc_output(timer_, timer_ocid_bw_);
  timer_set_oc_mode(timer_, timer_ocid_fw_, TIM_OCM_PWM1);
  timer_set_oc_mode(timer_, timer_ocid_bw_, TIM_OCM_PWM1);
  timer_enable_oc_output(timer_, timer_ocid_fw_);
  timer_enable_oc_output(timer_, timer_ocid_bw_);

  // Set TIMx_CCRx register (pwm duty). If the compare value in TIMx_CCRx is greater than the
  // auto-reload value in TIMx_ARR then OCxREF is held at 1. If the compare value is 0 then
  // OCxREF is held at 0 (STM32.388).
  timer_set_oc_value(timer_, timer_ocid_fw_, 0);
  timer_set_oc_value(timer_, timer_ocid_bw_, 0);
  timer_enable_counter(timer_);
}

//============================================= OPERATIONS =========================================

void PwmMotor2Wire::setSpeed(uint16_t speed, uint8_t forward)
{
  if (speed != 0) {
    if (forward) {
      timer_set_oc_value(timer_, timer_ocid_fw_, max_duty_);
      timer_set_oc_value(timer_, timer_ocid_bw_, speed);
    } else {
      timer_set_oc_value(timer_, timer_ocid_bw_, max_duty_);
      timer_set_oc_value(timer_, timer_ocid_fw_, speed);
    }
  } else {
    timer_set_oc_value(timer_, timer_ocid_fw_, max_duty_);
    timer_set_oc_value(timer_, timer_ocid_bw_, max_duty_);
    //timer_set_oc_value(timer_, timer_ocid_fw_, 0);
    //timer_set_oc_value(timer_, timer_ocid_bw_, 0);
  }
#if 0
  // AVR 2-wire motor
  //
  if (speed != 0) {
    if (reverse) {
      *ocr_fw_ = speed;
      *ocr_bw_ = max_pwm_;
    } else {
      *ocr_fw_ = max_pwm_;
      *ocr_bw_ = speed;
    }
  } else {
    *ocr_fw_ = max_pwm_;
    *ocr_bw_ = max_pwm_;
  }
#endif
}

uint16_t PwmMotor2Wire::max_duty() const
{
  return max_duty_;
}

} // namespace btr

#endif // _btr_PwmMotor2Wire_hpp_
