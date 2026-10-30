#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD     0U
#define CAN_ID_EXT     1U
#define CAN_RTR_DATA   0U

void Error_Handler(void);

#endif /* MAIN_H */
