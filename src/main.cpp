#include <Arduino.h>
#define DISABLE_CODE_FOR_RECEIVER
#define SEND_PWM_BY_TIMER
#define IR_USE_AVR_TIMER2
#include <IRremote.hpp>
#include <TimerOne.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define IR_LED_PIN (PD3)
#define LED_FEEDBACK_PIN (PD5)
#define TICK_INTERVAL (30000)
#define IDLE_TICKS_TIMEOUT (5)

/*
PWR   - PD0
CH+   - PD1
CH-   - PD2
VOL-  - PD4
VOL+  - PD6
*/

namespace Buttons
{
  enum
  {
    BTN_PWR,
    CH_PLUS,
    CH_MINUS,
    VOL_MINUS,
    VOL_PLUS,
    COUNT_
  };
}

typedef void (*SendFunc)(uint16_t, uint16_t);

struct ButtonHandler
{
  SendFunc send;
  uint16_t addr;
  uint16_t cmd;
};

void send_rc5(uint16_t addr, uint16_t cmd)
{
  IrSender.sendRC5((uint8_t)addr, (uint8_t)cmd, 0);
}

void send_samsung(uint16_t addr, uint16_t cmd)
{
  IrSender.sendSamsung(addr, cmd, 0);
}

volatile uint8_t button_pins[Buttons::COUNT_] = {PD0, PD1, PD2, PD4, PD6};
volatile uint8_t buttons_prev_state[Buttons::COUNT_] = {1, 1, 1, 1, 1};
volatile uint8_t buttons_state[Buttons::COUNT_] = {1, 1, 1, 1, 1};
volatile uint8_t tick = 0;
volatile uint8_t ticks_since_last_event = 0;
ButtonHandler button_handlers[Buttons::COUNT_] = {
    {send_samsung, 0x707, 0x2},
    {send_rc5, 0xE, 0x3C},
    {send_rc5, 0xE, 0x11},
    {send_rc5, 0xE, 0x13},
    {send_rc5, 0xE, 0x12},
};

void inc_tick()
{
  tick++;
  ticks_since_last_event++;
}

void setup()
{
  // disable ADC
  ADCSRA = 0;

  // turn off various modules
  power_all_disable();

  // but enable the necessary ones
  power_timer0_enable();
  power_timer1_enable();
  power_timer2_enable();

  // set input with pull-up for all ports
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  PORTB = 255;
  PORTC = 255;
  PORTD = 255;

  // IR Led pin should be OUTPUT
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, LOW);

  // Feedback pin is OUTPUT as well
  pinMode(LED_FEEDBACK_PIN, OUTPUT);
  digitalWrite(LED_FEEDBACK_PIN, LOW);

  // enable interrupts for PD
  PCICR |= 0b00000100;

  // enable interrupts for D0, D1, D2, D4, D6
  PCMSK2 |= 0b01010111;

  IrSender.begin(true, LED_FEEDBACK_PIN);
  Timer1.initialize(TICK_INTERVAL);
  Timer1.attachInterrupt(inc_tick);
}

void power_down()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
}

void idle()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
}

void read_pins()
{
  for (uint8_t i = 0; i < Buttons::COUNT_; i++)
  {
    buttons_state[i] = digitalRead(button_pins[i]);
  }
}

void update_previous_state()
{
  for (uint8_t i = 0; i < Buttons::COUNT_; i++)
  {
    buttons_prev_state[i] = buttons_state[i];
  }
}

uint8_t get_released_button()
{
  for (uint8_t i = 0; i < Buttons::COUNT_; i++)
  {
    if (buttons_prev_state[i] == 0 && buttons_state[i] == 1)
    {
      return i;
    }
  }
  return Buttons::COUNT_;
}

void loop()
{
  if (tick)
  {
    cli();
    tick = 0;
    read_pins();
    uint8_t button_pressed = get_released_button();
    update_previous_state();
    digitalWrite(LED_FEEDBACK_PIN, LOW);
    if (button_pressed != Buttons::COUNT_)
    {
      sei();
      Timer1.stop();
      ButtonHandler *handler_ptr = &button_handlers[button_pressed];
      handler_ptr->send(handler_ptr->addr, handler_ptr->cmd);
      ticks_since_last_event = 0;
      digitalWrite(LED_FEEDBACK_PIN, HIGH);
      Timer1.start();
    }
    sei();
    if (ticks_since_last_event >= IDLE_TICKS_TIMEOUT)
    {
      power_down();
    }
  }
  idle();
}

EMPTY_INTERRUPT(PCINT2_vect);
