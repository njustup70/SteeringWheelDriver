#ifndef MAIN_H
#define MAIN_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    const void *Instance;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

#define CAN_ID_STD  0U
#define CAN_ID_EXT  1U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 1U
#define DISABLE     0U

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox);

void Error_Handler(void);

#endif /* MAIN_H */
