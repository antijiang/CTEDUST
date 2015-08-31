#ifndef __SERIALTRANSFER_H__
#define __SERIALTRANSFER_H__

#include "M051Series.h"

#define SERIAL_BUFF_SIZE        (16)            // 串口缓冲区大小
#define START_BYTE              (0x01)          // 起始符
#define END_BYTE                (0x03)          // 结束符
#define ESC_BYTE                (0x02)          // 转义符

uint8_t GetDataLength(uint8_t *pu8EscData);
_Bool TranslateData(uint8_t *pu8EscData, uint8_t *pu8Data);
_Bool CheckDATA(uint8_t *pu8Data);

#endif

