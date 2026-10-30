#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

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

typedef struct
{
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

extern const int GM6020_STUB_CAN1;
extern const int GM6020_STUB_CAN2;

#define CAN1 (&GM6020_STUB_CAN1)
#define CAN2 (&GM6020_STUB_CAN2)

#define CAN_ID_STD                    0U
#define CAN_RTR_DATA                  0U
#define DISABLE                       0U
#define ENABLE                        1U
#define CAN_FILTERMODE_IDMASK         0U
#define CAN_FILTERSCALE_32BIT         0U
#define CAN_RX_FIFO0                  0U
#define CAN_IT_RX_FIFO0_MSG_PENDING   1U

HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan,
    CAN_FilterTypeDef *filter);

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef HAL_CAN_ActivateNotification(
    CAN_HandleTypeDef *hcan,
    uint32_t notification);

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox);

void Error_Handler(void);

#endif /* MAIN_H */
