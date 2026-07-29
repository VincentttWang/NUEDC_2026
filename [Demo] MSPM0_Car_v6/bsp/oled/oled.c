/**
 * @file  oled.c
 * @brief SSD1306 OLED 硬件 I2C 显示实现。
 */

#include "oled.h"
#include "oled_font.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stdint.h>

#define OLED_CMD  0U
#define OLED_DATA 1U
#define OLED_I2C_ADDR_7BIT 0x3CU
#define OLED_I2C_TIMEOUT   1000000U

static bool OLED_I2C_WaitIdle(void);
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len);
static void OLED_Write_Bytes(uint8_t control, const uint8_t *data, uint16_t len);
static void OLED_WR_Byte(uint8_t dat, uint8_t mode);
static void OLED_Write_ContinuousCmd(const uint8_t *cmds, uint8_t len);
static void OLED_Write_ContinuousData(const uint8_t *data, uint16_t len);
static void OLED_Set_Pos(uint8_t x, uint8_t y);
static uint32_t oled_pow(uint8_t m, uint8_t n);

static void OLED_DelayMs(uint32_t ms){
    BSP_DelayMs(ms);
}

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127
 
 
//反显函数
void OLED_ColorTurn(uint8_t i){
    if(i==0){
        OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
    }
    if(i==1){
        OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
    }
}
 
////屏幕旋转180度
//void OLED_DisplayTurn(uint8_t i)
//{
//if(i==0)
//    {
//        OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
//        OLED_WR_Byte(0xA1,OLED_CMD);
//    }
//    if(i==1)
//    {
//        OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
//        OLED_WR_Byte(0xA0,OLED_CMD);
//    }
//}
static bool OLED_I2C_WaitIdle(void){
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            return false;
        }
    }

    return true;
}

/* 使用阻塞轮询完成一次 I2C 写事务，调用方负责保证 data 在事务期间有效。 */
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len){
    uint16_t sent = 0U;
    uint32_t timeout;

    if (data == NULL || len == 0U){
        return true;
    }

    if (!OLED_I2C_WaitIdle()){
        return false;
    }

    DL_I2C_resetControllerTransfer(OLED_INST);

    while (sent < len && !DL_I2C_isControllerTXFIFOFull(OLED_INST)){
        DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
    }

    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    DL_Common_delayCycles(3U);

    timeout = OLED_I2C_TIMEOUT;
    while (sent < len){
        uint32_t status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
            return false;
        }
        if (!DL_I2C_isControllerTXFIFOFull(OLED_INST)){
            DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
            timeout = OLED_I2C_TIMEOUT;
        }
        else if (timeout-- == 0U){
            return false;
        }
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){
            return false;
        }
    }

    return ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U);
}

/* SSD1306 每段数据前都需要控制字节；分段可避免长页写入超过硬件 TX FIFO。 */
static void OLED_Write_Bytes(uint8_t control, const uint8_t *data, uint16_t len){
    uint8_t packet[9];
    uint16_t offset = 0U;

    if (data == NULL || len == 0U){
        return;
    }

    packet[0] = control;
    while (offset < len){
        uint8_t chunk_len = (uint8_t)((len - offset) > 8U ? 8U : (len - offset));
        for (uint8_t i = 0U; i < chunk_len; i++){
            packet[(uint8_t)(i + 1U)] = data[(uint16_t)(offset + i)];
        }

        if (!OLED_I2C_Write(packet, (uint16_t)(chunk_len + 1U))){
            return;
        }
        offset = (uint16_t)(offset + chunk_len);
    }
}

//发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
static void OLED_WR_Byte(uint8_t dat,uint8_t mode){
    OLED_Write_Bytes((mode != 0U) ? 0x40U : 0x00U, &dat, 1U);
}
 
static void OLED_Write_ContinuousCmd(const uint8_t *cmds, uint8_t len){
    OLED_Write_Bytes(0x00U, cmds, len);
}
 
//坐标设置
static void OLED_Set_Pos(uint8_t x, uint8_t y){
//    OLED_WR_Byte(0xb0+y,OLED_CMD);
//    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
//    OLED_WR_Byte((x&0x0f),OLED_CMD);
    uint8_t cmds[3];
    cmds[0] = 0xb0 + y;                  // 页地址
    cmds[1] = ((x & 0xf0) >> 4) | 0x10;  // 列高地址
    cmds[2] = x & 0x0f;                  // 列低地址
    OLED_Write_ContinuousCmd(cmds, 3);   // 一次传输3个命令
}
 
//开启OLED显示    
void OLED_DisplayOn(void){
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
 
void OLED_Display_On(void){
    OLED_DisplayOn();
}

//关闭OLED显示     
void OLED_DisplayOff(void){
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}

void OLED_Display_Off(void){
    OLED_DisplayOff();
}

static void OLED_Write_ContinuousData(const uint8_t *data, uint16_t len){
    OLED_Write_Bytes(0x40U, data, len);
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void){  
//    uint8_t i,n;		    
//    for(i=0;i<8;i++)  
//    {  
//        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
//        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
//        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
//        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
//    } //更新显示
    static uint8_t clear_buf[128] = {0};  // 静态缓冲区，避免每次初始化
    // 循环8页，每页设置位置后连续写128字节0
    for (uint8_t i = 0; i < 8; i++){  
        OLED_Set_Pos(0, i);  // 优化后：一次I2C传输3个命令
        OLED_Write_ContinuousData(clear_buf, 128);
    }
}

void OLED_ClearLine(uint8_t y){
    static uint8_t clear_buf[128] = {0};

    if (y >= 8){
        return;
    }

    OLED_Set_Pos(0, y);
    OLED_Write_ContinuousData(clear_buf, 128);
}
 
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63				 
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey){      	
    uint8_t c=0,sizex=sizey/2;
    uint16_t i=0,size1;

    if (sizey != 8 && sizey != 16){
        return;
    }

    if (y >= 8 || x >= 128){
        return;
    }

    if (chr < ' ' || chr > '~'){
        chr = '?';
    }

    if(sizey==8){
        size1=6;
    }
    else{
        size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    }
    if ((uint16_t)x + sizex > 128){
        return;
    }

    c=chr-' ';//得到偏移后的值
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++){
        if(i%sizex==0&&sizey!=8){
            OLED_Set_Pos(x,y++);
        }
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8字号
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16字号
        //		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//用户添加字号
        else{
            return;
        }
    }
}
 
//m^n函数
static uint32_t oled_pow(uint8_t m,uint8_t n){
    uint32_t result=1;	 
    while(n--){
        result*=m;
    }
    return result;
}
 
//显示数字
//x,y :起点坐标
//num:要显示的数字
//len :数字的位数
//sizey:字体大小		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey){         	
    uint8_t t,temp,m=0;
    uint8_t enshow=0;
    if(sizey==8){
        m=2;
    }
    for(t=0;t<len;t++){
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1)){
            if(temp==0){
                OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
                continue;
            }else enshow=1;
        }
        OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
    }
}
 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t sizey){
    uint8_t j=0;
    uint8_t step = (sizey == 8) ? 6 : (sizey / 2);

    if (chr == NULL || step == 0){
        return;
    }

    while (chr[j]!='\0' && x < 128){		
        if ((uint16_t)x + step > 128){
            break;
        }
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8){
            x+=6;
        }
        else{
            x+=sizey/2;
        }
    }
}

void OLED_ShowStringClearLine(uint8_t x,uint8_t y,const char *chr,uint8_t sizey){
    OLED_ClearLine(y);
    OLED_ShowString(x, y, chr, sizey);
}
 
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t *bmp){
    if (bmp == NULL || x >= OLED_WIDTH || y >= OLED_PAGE_COUNT || sizex == 0U || sizey == 0U){
        return;
    }

    uint8_t page_count = (uint8_t)((sizey + 7U) / 8U);
    uint16_t bmp_index = 0U;

    for (uint8_t page = 0U; page < page_count && (uint8_t)(y + page) < OLED_PAGE_COUNT; page++){
        OLED_Set_Pos(x, (uint8_t)(y + page));

        for (uint8_t col = 0U; col < sizex && (uint8_t)(x + col) < OLED_WIDTH; col++){
            OLED_WR_Byte(bmp[bmp_index + col], OLED_DATA);
        }

        bmp_index = (uint16_t)(bmp_index + sizex);
    }
}


//初始化SSD1306					    
void OLED_Init(void){
    OLED_DelayMs(100);
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
} 
