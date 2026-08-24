/**
 * @file    app_tasks.h
 * @brief   FreeRTOS task definitions, priorities, and stack sizes.
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

/*---------------------------------------------------------------------------
 * Task priorities (0 = idle, higher = more urgent)
 *---------------------------------------------------------------------------*/

#define PRIO_COMM_TASK          (tskIDLE_PRIORITY + 3)
#define PRIO_CONTROL_TASK       (tskIDLE_PRIORITY + 2)
#define PRIO_APP_TASK           (tskIDLE_PRIORITY + 1)
#define PRIO_MONITOR_TASK       (tskIDLE_PRIORITY + 1)

/*---------------------------------------------------------------------------
 * Stack sizes (in words, not bytes)
 *---------------------------------------------------------------------------*/

#define STACK_COMM_TASK         768U
#define STACK_CONTROL_TASK      512U
#define STACK_APP_TASK          768U
#define STACK_MONITOR_TASK      256U

/*---------------------------------------------------------------------------
 * Queue sizes
 *---------------------------------------------------------------------------*/

#define QUEUE_CMD_LENGTH        8U
#define QUEUE_DATA_LENGTH       16U

/*---------------------------------------------------------------------------
 * Initialisation
 *---------------------------------------------------------------------------*/

/**
 * @brief Create all application tasks and communication primitives.
 *        Called from main() before vTaskStartScheduler().
 */
void app_tasks_init(void);

#endif /* APP_TASKS_H */
