#ifndef DRV_CAN_H
#define DRV_CAN_H

typedef struct stm32_can
{
	CAN_HandleTypeDef hcan;

}stm32_canTypeDef;

#define  CAN1_CONFIG     	\
{							\
    250, 	/*250kbps*/		\
    64,						\
    0,						\
    0,						\
    0,						\
    0,						\
    10						\
}				

#endif
