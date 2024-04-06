#include <driver/gpio.h>
#include <driver/rtc_io.h>

#define GPIO_LED_GREEN GPIO_NUM_18 /* Briefly flash if door is closed */
#define GPIO_LED_RED GPIO_NUM_19   /* Briefly flash if door is open */
#define GPIO_SWITCH GPIO_NUM_15    /* Door switch connected to this GPIO */
#define SLEEP_FOR_US (20 * 1000000)

void setup() {
  Serial.begin(115200);

  /* Print switch status */
  Serial.print("Switch status: ");
  Serial.println(digitalRead(GPIO_SWITCH));

  /* LED outputs. */
  gpio_set_direction(GPIO_LED_GREEN, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_LED_RED, GPIO_MODE_OUTPUT);

  // gpio_set_direction(GPIO_SWITCH, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(GPIO_SWITCH, GPIO_PULLUP_ONLY);

  rtc_gpio_deinit(GPIO_SWITCH);

  print_wakeup_cause();

  const int pin_state = digitalRead(GPIO_SWITCH);
  flash_led(pin_state);

  /* Make sure to pull up the sensor input. */
  rtc_gpio_pullup_en(GPIO_SWITCH);

  /* Set up the conditions for waking up. */
  esp_sleep_enable_ext0_wakeup(GPIO_SWITCH, pin_state ^ 1);

  /* Set up timer wakeup for predefined timeout. */
  esp_sleep_enable_timer_wakeup(SLEEP_FOR_US);

  Serial.println("ZZZzzz");
  esp_deep_sleep_start();
}

void print_wakeup_cause() {
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

  switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wake up from GPIO pin change.");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wake up from timer timeout.");
      break;
    default:
      Serial.println("Wake up from unknown source.");
      break;
  }
}

void flash_led(const int pin_state) {
  gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 1);
  gpio_set_level(pin_state ? GPIO_LED_GREEN : GPIO_LED_RED, 0);
  delay(50);
  // gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 0);
}

void loop() {
  // const int pin_state = digitalRead(GPIO_SWITCH);
  // gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 1);
  // gpio_set_level(pin_state ? GPIO_LED_GREEN : GPIO_LED_RED, 0);
  // delay(5000);
  // flash_led(pin_state);
}
