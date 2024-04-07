#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include "config.h"

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

  print_wakeup_cause();

  const int pin_state = digitalRead(GPIO_SWITCH);
  flash_led(pin_state);
  bool success = mqtt_update(pin_state);
  if (success) {
    // Call log_pin_state only if mqtt_update was successful
    log_pin_state(pin_state);
  }
  flash_led(pin_state);

  rtc_gpio_deinit(GPIO_SWITCH);

  /* Make sure to pull up the sensor input. */
  rtc_gpio_pullup_en(GPIO_SWITCH);

  /* Set up the conditions for waking up. */
  esp_sleep_enable_ext0_wakeup(GPIO_SWITCH, pin_state ^ 1);

  /* Set up timer wakeup for predefined timeout. */
  esp_sleep_enable_timer_wakeup(SLEEP_FOR_US);

  Serial.println("ZZZzzz");
  esp_deep_sleep_start();
}

void loop() {
  // const int pin_state = digitalRead(GPIO_SWITCH);
  // gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 1);
  // gpio_set_level(pin_state ? GPIO_LED_GREEN : GPIO_LED_RED, 0);
  // delay(5000);
  // flash_led(pin_state);
}

void flash_led(const int pin_state) {
  gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 1);
  gpio_set_level(pin_state ? GPIO_LED_GREEN : GPIO_LED_RED, 0);

  delay(50);
  // gpio_set_level(pin_state ? GPIO_LED_RED : GPIO_LED_GREEN, 0);
}

void print_wakeup_cause() {
  int wakeup_cause = esp_sleep_get_wakeup_cause();

  switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wake up from GPIO pin change.");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wake up from timer timeout.");
      break;
    default:
      Serial.println("Wake up from unknown reason.");
      break;
  }
}

bool wifi_connect() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  int retries = 3; // Number of connection attempts
  while (retries > 0 ) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      return true;  // Exit the function if connected successfully
    } else {
      Serial.println("Connection failed. Retrying... ");
      retries--;
    }
  }
  Serial.println("Failed to connect to WiFi. Going ack to sleep.");
  return false;
}

bool mqtt_connect(Adafruit_MQTT_Client& mqtt) {
  Serial.print("Connecting to MQTT… ");

  uint8_t retries = 3;
  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));

    if (retries-- == 0) {
      Serial.println("Stopping retries…");
      return false;
    }

    mqtt.disconnect();
    Serial.println("Retrying MQTT connection in 5 seconds…");
    delay(5000); // wait 5 seconds
  };

  return true;
}

bool mqtt_update(const int pin_state) {
  WiFiClient client;

  // Setup the MQTT client class by passing in the WiFi client and 
  // MQTT server and login details.
  Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

  // Setup a feed called‘feed’for publishing
  Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" AIO_FEED);

  if (!wifi_connect()) {
    Serial.println("Failed to connect to WiFi. Going back to sleep.");
    return false;
  }

  // Connect to MQTT
  if (!mqtt_connect(mqtt)) {
    Serial.println("MQTT Not Connected! Going back to sleep.");
    return false;
  }

  Serial.println("Connected to WiFi and MQTT!");

  // Publish pin state to MQTT
  if (!feed.publish((uint32_t)pin_state)) {
    Serial.println(F("Failed to publish pin state to MQTT"));
    return false;
  } else {
    Serial.println(F("Pin state published to MQTT successfully!"));
    }
  return true;
}

void log_pin_state(const int pin_state) {
  Serial.print("Pin state: ");
  Serial.println(pin_state);
}





