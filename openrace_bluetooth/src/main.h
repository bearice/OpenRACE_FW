#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern xQueueHandle event_queue;

typedef struct QueueMessage {
  uint8_t buf[8];
} QueueMessage_t;
