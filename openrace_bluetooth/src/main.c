#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/dport_reg.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"
#include "sdkconfig.h"
#include "main.h"

#define TAG "MAIN"
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                        /*!< Data buffer length of data buffer */
#define I2C_SLAVE_SCL_IO 25                    /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 26                    /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(0)            /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size */
#define ESP_SLAVE_ADDR 4                       /*!< ESP32 slave address, you can set any 7bit value */

// static xQueueHandle i2c_event_queue = NULL;

// struct QueueMessage {
//   int interruptNo;
// } xMessage;

// static intr_handle_t i2c_slave_intr_handle = NULL;
// static void IRAM_ATTR i2c_onData(void *arg);
// static void IRAM_ATTR i2c_slave_isr_handler_default(void *arg);
/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init() {
  int i2c_slave_port = I2C_SLAVE_NUM;
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
  i2c_param_config(i2c_slave_port, &conf_slave);

  // if (i2c_isr_register(I2C_SLAVE_NUM, i2c_onData, NULL, 0, &i2c_slave_intr_handle) != ESP_OK) {
  //   ESP_LOGE(TAG, "i2c_isr_register error");
  //   return ESP_FAIL;
  // }

  i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
  return ESP_OK;
}
/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len) {
  int i;
  for (i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

// static void IRAM_ATTR i2c_onData(void *arg) {
//   ets_printf("int_status %d\n", I2C0.int_status.val);
//   if (I2C0.int_status.end_detect) {
//     struct QueueMessage *message;
//     message = (struct QueueMessage *)malloc(sizeof(struct QueueMessage));
//     message->interruptNo = esp_intr_get_intno(i2c_slave_intr_handle);

//     if (i2c_isr_free(i2c_slave_intr_handle) == ESP_OK) {
//       i2c_slave_intr_handle = NULL;
//       ets_printf("Free-ed interrupt handler\n");
//     } else {
//       ets_printf("Failed to free interrupt handler\n");
//     }
//     BaseType_t ret = xQueueSendFromISR(i2c_event_queue, &message, NULL);
//     if (ret != pdTRUE) {
//       ets_printf("Could not send event to queue (%d)\n", ret);
//     }
//   }
// }

extern uint8_t battary_lev;
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_7;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

static void check_efuse() {
  //Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    printf("eFuse Two Point: Supported\n");
  } else {
    printf("eFuse Two Point: NOT supported\n");
  }

  //Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    printf("eFuse Vref: Supported\n");
  } else {
    printf("eFuse Vref: NOT supported\n");
  }
}

static void print_char_val_type(esp_adc_cal_value_t val_type) {
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    printf("Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    printf("Characterized using eFuse Vref\n");
  } else {
    printf("Characterized using Default Vref\n");
  }
}

#define BATT_MIN_VOLT 2700
#define BATT_MAX_VOLT 4200

int get_battery_level() {
  uint32_t adc_reading = 0;
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw(channel);
  }
  adc_reading /= NO_OF_SAMPLES;
  //Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) * 2;
  uint32_t lvl = (voltage - BATT_MIN_VOLT) * 100 / (BATT_MAX_VOLT - BATT_MIN_VOLT);
  printf("Raw: %d\tVoltage: %dmV\tLevel: %d%%\n", adc_reading, voltage, lvl);
  return lvl;
}

static void adc_battary_task(void *arg) {
  while (1) {
    battary_lev = get_battery_level();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void i2c_slave_task(void *arg) {
  ESP_LOGI(TAG, "Starting i2c_slave_task loop");
  uint8_t buf[I2C_SLAVE_RX_BUF_LEN] = {0};
  while (1) {
    // struct QueueMessage *message = NULL;
    // BaseType_t ret = xQueueReceive(i2c_event_queue, &message, 1000 / portTICK_RATE_MS);
    // if (ret) {
    // free(message);
    uint8_t type;
    int ret = i2c_slave_read_buffer(I2C_SLAVE_NUM, &type, 1, 1000 / portTICK_RATE_MS);
    if (ret) {
      printf("Got message type %d\n", ret);
      switch (type) {
      case 1:
        ret = i2c_slave_read_buffer(I2C_SLAVE_NUM, buf, 4, 1000 / portTICK_RATE_MS);
        printf("Heartbeat message: \n");
        disp_buf(buf, ret);
        // adc_read_val();
        break;
      case 2:
        ret = i2c_slave_read_buffer(I2C_SLAVE_NUM, buf, 8, 1000 / portTICK_RATE_MS);
        printf("Report message: \n");
        disp_buf(buf, ret);
        QueueMessage_t *msg = malloc(sizeof(QueueMessage_t));
        memcpy(msg->buf, buf, 8);
        xQueueSend(event_queue, &msg, 1000 / portTICK_RATE_MS);
      }
    }

    //   esp_err_t isr_register_ret = i2c_isr_register(I2C_SLAVE_NUM, i2c_onData, 0, 0, &i2c_slave_intr_handle);
    //   if (isr_register_ret == ESP_OK) {
    //     ESP_LOGI(TAG, "Registered interrupt handler");
    //   } else {
    //     ESP_LOGW(TAG, "Failed to register interrupt handler");
    //   }
    // } else {
    //   ESP_LOGW(TAG, "Failed to get queued event");
    //   printf("xQueueReceive() returned %d\n", ret);
    // }
    // vTaskDelay(portTICK_RATE_MS / 1000);
  }
}

xQueueHandle event_queue;
void ble_hid_init();
void app_main() {
  printf("Hello world!\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  printf("silicon revision %d, ", chip_info.revision);

  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  fflush(stdout);

  // for (int i = 10; i >= 0; i--) {
  //   printf("Restarting in %d seconds...\n", i);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
  // printf("Restarting now.\n");
  // fflush(stdout);
  // esp_restart();
  event_queue = xQueueCreate(5, sizeof(QueueMessage_t *));

  if (event_queue == 0) {
    ESP_LOGW(TAG, "Failed to create event queue");
  }

  check_efuse();
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(channel, atten);
  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);

  i2c_slave_init();
  xTaskCreate(i2c_slave_task, "i2c_slave_task", 1024 * 2, (void *)0, 10, NULL);
  xTaskCreate(adc_battary_task, "adc_battary_task", 1024 * 2, (void *)0, 10, NULL);
  ble_hid_init();
}