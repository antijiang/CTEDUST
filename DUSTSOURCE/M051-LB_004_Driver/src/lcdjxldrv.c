#include "M051Series.h"
#include "LCD_Driver.h"
#define	PIN_cs1 P04
#define PIN_rs	P06
#define PIN_sclk	P07
#define	PIN_sid	P05
#define PIN_reset	P32
const uint8_t  ascii_table_8x16[95][16];
void  display_string_8x16(unsigned char line,char column,char *str) ;
void delay(int i);


//端口为输出功能，cs ，rs data sck，
void Init_LCDPio()
{
	P0->PMD=(1<<GPIO_PMD_PMD7_Pos)|(1<<GPIO_PMD_PMD6_Pos)|(1<<GPIO_PMD_PMD5_Pos)|(1<<GPIO_PMD_PMD4_Pos);
}
void Set_PINCS(uint8_t hlevel)
{
	delay(1);
	if(hlevel)PIN_cs1=1;
	else PIN_cs1=0;
	delay(1);
	
}
void Byte_send(uint8_t data1 )
{
uint8_t i;
for(i=0;i<8;i++)
{
PIN_sclk=0;
if(data1&0x80) PIN_sid=1;
else PIN_sid=0;
delay(1);
PIN_sclk=1;
data1=data1<<=1;
delay(1);
}
}
/*写指令到LCD 模块*/
void transfer_command(int data1)
{

Set_PINCS(0);
PIN_rs=0;

Byte_send(data1);
Set_PINCS(1);

}
/*写数据到LCD 模块*/
void transfer_data(int data1)
{

Set_PINCS(0);
PIN_rs=1;
Byte_send(data1);
Set_PINCS(0);
}
/*延时*/
void delay(int i)
{
int j,k;
	
for(j=0;j<i;j++)
for(k=0;k<10;k++);
}


/*LCD 模块初始化*/
void LCD_Init()
{
Set_PINCS(1);
	delay(100);
Set_PINCS(0);	
PIN_reset=0; /*低电平复位*/
delay(100);
PIN_reset=1; /*复位完毕*/
delay(20);
transfer_command(0xe2); /*软复位*/
delay(5);
transfer_command(0x2c); /*升压步聚1*/

delay(5);
transfer_command(0x2e); /*升压步聚2*/
delay(5);
transfer_command(0x2f); /*升压步聚3*/
delay(5);
transfer_command(0x23); /*粗调对比度，可设置范围0x20～0x27*/
transfer_command(0x81); /*微调对比度*/
transfer_command(0x28); /*0x1a,微调对比度的值，可设置范围0x00～0x3f*/

transfer_command(0xa2); /*1/9 偏压比（bias）*/
transfer_command(0xc8); /*行扫描顺序：从上到下*/
transfer_command(0xa0); /*列扫描顺序：从左到右*/
transfer_command(0x40); /*起始行：第一行开始*/
transfer_command(0xaf); /*开显示*/
Set_PINCS(1);
{
	void full_display();
 full_display();
}

display_string_8x16(0,0,"0123456789abcdef");/*在第1 页，第1 列显示字符串*/
}

void lcd_address(uint8_t page,uint8_t column)
{
//Set_PINCS(0);
column=column; 
transfer_command(0xb0+page); //设置页地址。每页是8 行。一个画面的64 行被分成8 个页。我们平常所说的第1 页，在LCD 驱动IC 里是第0 页，*/
transfer_command(((column>>4)&0x0f)+0x10); //设置列地址的高4 位
transfer_command(column&0x0f); //设置列地址的低4 位
	
//Set_PINCS(1);

}
/*全屏清屏*/
void LCD_ClearScreen(void)
{
uint8_t i,j;
//Set_PINCS(0);
for(i=0;i<8;i++)
{
lcd_address(i,0);
for(j=0;j<132;j++)
{
transfer_data(0x00);
}
}
//Set_PINCS(1);
}
//==================display a piture of 128*64 dots================
void full_display()
{
int i,j;
//Set_PINCS(0);
for(i=0;i<8;i++)
{

lcd_address(i,0);
for(j=0;j<128;j++)
{
transfer_data(0xff);
}
}
//Set_PINCS(1);
}


/*显示8x16 点阵图像、ASCII, 或8x16 点阵的自造字符、其他图标*/
void display_graphic_8x16(uint8_t page,uint8_t column,uint8_t *dp)
{
uint8_t i,j;
//Set_PINCS(0);
for(j=0;j<2;j++)
{
lcd_address(page+j,column);
for (i=0;i<8;i++)
{
transfer_data(*dp); /*写数据到LCD,每写完一个8 位的数据后列地址自动加1*/
dp++;
}
}
//Set_PINCS(1);
}


void LCD_Print(unsigned char  line, char *str)
{
   display_string_8x16( 2*line, 0, str);
}


void display_string_8x16(unsigned char page,char column,char *text)
{
uint16_t i=0,j,k,n;
//Set_PINCS(0);
	
while(text[i]>0x00)
{
if((text[i]>=0x20)&&(text[i]<=0x7e))
{
j=text[i]-0x20;
for(n=0;n<2;n++)
{
lcd_address(page+n,column);
for(k=0;k<8;k++)
{
transfer_data(ascii_table_8x16[j][k+8*n]);/*??5x7 ?ASCII ??LCD ?,y ????,x ????,?????*/
}
}
i++;
column+=8;
}
else
i++;
}

//Set_PINCS(1);
}



void LCD_EnableBackLight(void)
{
    P11 = 1;
}
void LCD_DisableBackLight(void)
{
    P11 = 0;
}
const uint8_t  ascii_table_8x16[95][16]={
/*-- 文字: --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: ! --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,
/*-- 文字: " --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: # --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,
/*-- 文字: $ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,

/*-- 文字: % --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,
/*-- 文字: & --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,
/*-- 文字: ' --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: ( --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,
/*-- 文字: ) --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,
/*-- 文字: * --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,
/*-- 文字: + --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,
/*-- 文字: , --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: - --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
/*-- 文字: . --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: / --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,
/*-- 文字: 0 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,
/*-- 文字: 1 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,
/*-- 文字: 2 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,
/*-- 文字: 3 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,
/*-- 文字: 4 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,
/*-- 文字: 5 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,
/*-- 文字: 6 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,

/*-- 文字: 7 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,
/*-- 文字: 8 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,
/*-- 文字: 9 --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,
/*-- 文字: : --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,
/*-- 文字: ; --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,
/*-- 文字: < --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,
/*-- 文字: = --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,
/*-- 文字: > --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,
/*-- 文字: ? --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,
/*-- 文字: @ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,
/*-- 文字: A --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,
/*-- 文字: B --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,
/*-- 文字: C --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,
/*-- 文字: D --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,
/*-- 文字: E --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,
/*-- 文字: F --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,
/*-- 文字: G --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,
/*-- 文字: H --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,
/*-- 文字: I --*/

/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,
/*-- 文字: J --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,
/*-- 文字: K --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,
/*-- 文字: L --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,
/*-- 文字: M --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,
/*-- 文字: N --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,
/*-- 文字: O --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,
/*-- 文字: P --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,
/*-- 文字: Q --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,
/*-- 文字: R --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,
/*-- 文字: S --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,
/*-- 文字: T --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,
/*-- 文字: U --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,
/*-- 文字: V --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,
/*-- 文字: W --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,
/*-- 文字: X --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,
/*-- 文字: Y --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,
/*-- 文字: Z --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,
/*-- 文字: [ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/

0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,
/*-- 文字: \ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,
/*-- 文字: ] --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,
/*-- 文字: ^ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: _ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
/*-- 文字: ` --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/*-- 文字: a --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,
/*-- 文字: b --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,
/*-- 文字: c --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,
/*-- 文字: d --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,
/*-- 文字: e --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,
/*-- 文字: f --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,
/*-- 文字: g --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,
/*-- 文字: h --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,
/*-- 文字: i --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,
/*-- 文字: j --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,
/*-- 文字: k --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,
/*-- 文字: l --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,
/*-- 文字: m --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,

/*-- 文字: n --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,
/*-- 文字: o --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,
/*-- 文字: p --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,
/*-- 文字: q --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,
/*-- 文字: r --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,
/*-- 文字: s --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,
/*-- 文字: t --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,
/*-- 文字: u --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,
/*-- 文字: v --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,
/*-- 文字: w --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,
/*-- 文字: x --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,
/*-- 文字: y --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,
/*-- 文字: z --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,
/*-- 文字: { --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,
/*-- 文字: | --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,
/*-- 文字: } --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,
/*-- 文字: ~ --*/
/*-- Comic Sans MS12; 此字体下对应的点阵为：宽x 高=8x16 --*/
0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
