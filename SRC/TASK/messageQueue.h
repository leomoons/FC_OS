#ifndef __MESSAGE_QUEUE_H__
#define __MESSAGE_QUEUE_H__

#include "TaskConfig.h"

extern QueueHandle_t messageQueue[QUEUE_NUM];

void MessageQueueCreate(void);

#endif
