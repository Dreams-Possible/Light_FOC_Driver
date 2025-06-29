#pragma once

//头文件
#include"sp_sys.h"
#include<stdio.h>
#include<string.h>
#include"driver/uart.h"

//初始化串口
uint8_t sp_uart_init();
//串口接收
char*sp_uart_rx();
//串口发送
void sp_uart_tx(char*data,...);
