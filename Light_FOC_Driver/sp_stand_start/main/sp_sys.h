#pragma once

#include<stdint.h>
#include"sp_app.h"
#include"sp_uart.h"
#include"esp_log.h"

typedef uint8_t u8;
typedef int8_t i8;
typedef uint16_t u16;
typedef int16_t i16;
typedef uint32_t u32;
typedef int32_t i32;
typedef u8 sp_t;
#define SP_OK 0
#define SP_FAIL 1
#define SP_LOG(fmt,...) printf("[SP_INFO][%s:%d:%s]: "fmt"\n",__FILE__,__LINE__,__func__,##__VA_ARGS__)
