/* ============================================================================
 *  oled.c —— SSD1306 OLED(7 针 SPI)驱动
 * ----------------------------------------------------------------------------
 *  接口区分(和 I2C 版的唯一区别就在最下面那一段低层):
 *      SPI : SCLK=PB9(D0), MOSI=PB8(D1)   —— 由 SysConfig 的 SPI1 实例 "OLED" 提供
 *      GPIO: RES=PB10, DC=PB11, CS=PB14, BLK=PB26
 *            —— 普通输出, 由 SysConfig 的 "oled" 实例提供
 *      其余 VCC/GND 接电源
 *
 *  协议要点(SSD1306 4-wire SPI):
 *      CS  低电平期间有效
 *      DC  低 = 下一条字节是命令; 高 = 数据
 *      SCLK 空闲低, 上升沿采样(SPI 模式0) —— SysConfig 里配的是 MOTO4_POL0_PHA0, 对得上
 *
 *  本文件由参考工程(I2C 版)移植: 上层(显存/画点/字符/汉字/图片)完全沿用,
 *  只把 OLED_WR_Byte() 从"硬件 I2C + 控制字节 0x00/0x40"改成"DC 脚 + SPI"。
 * ==========================================================================*/
#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/* ============================================================================
 *  屏幕适配参数(SH1106 / SSD1306)
 * ----------------------------------------------------------------------------
 *  ⚠ 1.3 寸 OLED 绝大多数是 SH1106, 不是 SSD1306! 两者最大区别:
 *      SH1106 显存有 132 列, 而可见的 128 列是从"第 2 列"开始的,
 *      所以写显存时列地址必须加偏移 2, 否则整体错位、右边出现花点/缺块。
 *    另外 SH1106 没有 SSD1306 的 0x20(寻址模式) 命令, 电荷泵命令也不同。
 * ==========================================================================*/
/* 列偏移: SH1106 = 2, SSD1306 = 0
 * 依据(官方 1.3寸 A1-1-3H 例程):
 *   OLED_Init()     里写的是 0x00 / 0x10   <- 看不出偏移
 *   OLED_Refresh()  里写的是 0x02 / 0x10   <- 真正的偏移 = 2 !!
 * 因为每次刷屏都会重设列地址, Init 里的值会被覆盖, 所以只看初始化会误判。
 * 偏移 2 = SH1106(显存 132 列, 可见区从第 2 列开始)的特征。
 * 做成变量是为了能按 KEY1 在运行期切换, 万一不是这型号也能试出来。 */
u8 g_oled_col_offset = 2U;   /* ★ 已实测确认: 偏移 2 时四边完整, 0/1 左边缺, 3 右边缺 */

/* 显示方向: 这两个值互换即可翻转对应方向
 *   0xA0 / 0xA1 : 左右(段重映射) — 0xA1 是常见"正常"值
 *   0xC0 / 0xC8 : 上下(COM 扫描) — 0xC8 是常见"正常"值
 *   整屏转了 180° => 两个都取反 */
#define OLED_SEG_REMAP        0xA1     /* 官方 1.3寸例程用 0xA1(正常) */
#define OLED_COM_SCAN         0xC8     /* 官方 1.3寸例程用 0xC8(正常) */

/* 显存: 144 列(留出滚动余量) x 8 页, 每页 8 行 */
u8 OLED_GRAM[144][8];

/* ============================================================================
 *  低层: SPI + CS/DC/RES 控制
 * ==========================================================================*/
#define OLED_CS_LOW()     DL_GPIO_clearPins(oled_PORT, oled_CS_PIN)
#define OLED_CS_HIGH()    DL_GPIO_setPins(oled_PORT, oled_CS_PIN)
#define OLED_DC_CMD()     DL_GPIO_clearPins(oled_PORT, oled_DC_PIN)
#define OLED_DC_DATA()    DL_GPIO_setPins(oled_PORT, oled_DC_PIN)
#define OLED_RES_LOW()    DL_GPIO_clearPins(oled_PORT, oled_RES_PIN)
#define OLED_RES_HIGH()   DL_GPIO_setPins(oled_PORT, oled_RES_PIN)
#define OLED_BLK_ON()     DL_GPIO_setPins(oled_PORT, oled_BLK_PIN)
#define OLED_BLK_OFF()    DL_GPIO_clearPins(oled_PORT, oled_BLK_PIN)

/* 小写包装, 让 OLED_Refresh 里能像函数一样调用(便于整页连续发送) */
static void oled_cs_low (void) { OLED_CS_LOW();  }
static void oled_cs_high(void) { OLED_CS_HIGH(); }
static void oled_dc_cmd (void) { OLED_DC_CMD();  }
static void oled_dc_data(void) { OLED_DC_DATA(); }

/* 发一个字节: 只在 TX FIFO 满时等待(防止写溢出丢字节), 其余交给 FIFO 排队 */
static void oled_spi_write(u8 dat)
{
    while (DL_SPI_isTXFIFOFull(OLED_INST)) { }
    DL_SPI_transmitData8(OLED_INST, dat);
}

/* 收尾: 等数据真正发完, 再把回读的 dummy 字节清掉
 * 注意顺序: 必须先等 TX FIFO 排空 —— 刚写完时 BUSY 位还没置起来,
 * 直接等 !isBusy 会立刻通过, 造成"以为发完了其实没发"的竞态。 */
static void oled_spi_flush(void)
{
    while (!DL_SPI_isTXFIFOEmpty(OLED_INST)) { }   /* 1. 等 FIFO 排空 */
    while (DL_SPI_isBusy(OLED_INST)) { }           /* 2. 等最后一位移出 */
    while (!DL_SPI_isRXFIFOEmpty(OLED_INST)) { (void)DL_SPI_receiveData8(OLED_INST); }
}

/* 写命令/数据: I2C 版靠控制字节 0x00/0x40, SPI 版靠 DC 脚 */
void OLED_WR_Byte(u8 dat, u8 mode)
{
    if (mode == OLED_DATA) { OLED_DC_DATA(); }
    else                   { OLED_DC_CMD();  }

    OLED_CS_LOW();
    oled_spi_write(dat);
    oled_spi_flush();       /* ★ 必须在 CS 拉高前等这一字节真正发完 */
    OLED_CS_HIGH();
}

/* ============================================================================
 *  上层: 与 I2C 版完全一致
 * ==========================================================================*/
void OLED_ColorTurn(u8 i)
{
    if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);   //正常显示
    if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);   //反色显示
}

void OLED_DisplayTurn(u8 i)
{
    if(i==0) { OLED_WR_Byte(OLED_COM_SCAN,OLED_CMD);  OLED_WR_Byte(OLED_SEG_REMAP,OLED_CMD); }
    if(i==1) { OLED_WR_Byte((OLED_COM_SCAN==0xC8)?0xC0:0xC8,OLED_CMD);
               OLED_WR_Byte((OLED_SEG_REMAP==0xA1)?0xA0:0xA1,OLED_CMD); }
}

void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D,OLED_CMD);    //电荷泵使能
    OLED_WR_Byte(0x14,OLED_CMD);    //开启电荷泵
    OLED_WR_Byte(0xAF,OLED_CMD);    //点亮屏幕
}

void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D,OLED_CMD);    //电荷泵设置
    OLED_WR_Byte(0x10,OLED_CMD);    //关闭电荷泵
    OLED_WR_Byte(0xAE,OLED_CMD);    //关闭屏幕(0xAE=off, 参考工程这里写成 0xAF 是笔误)
}

/* ★ 位序: 页内 bit0 = 最上面一行, bit7 = 最下面一行
 *   (SSD1306/SH1106 的页寻址规定 D0 对应页首行; 官方例程也是 n=1<<m)
 *   这里写成 1<<(7-...) 的话, 每个 8 像素高的横条会被上下翻转 -> 全屏乱码! */
void OLED_ClearPoint(u8 x,u8 y)
{
    if(x>127||y>63) return;
    OLED_GRAM[x][y/8] &= (u8)~(1U << (y%8));
}

void OLED_DrawPoint(u8 x,u8 y)
{
    if(x>127||y>63) return;
    OLED_GRAM[x][y/8] |= (u8)(1U << (y%8));
}

/* 把显存刷到屏幕(页寻址) */
void OLED_Refresh(void)
{
    u8 i,n;
    for(i=0;i<8;i++)
    {
        /* 一整页(3 个命令 + 128 字节数据)只用一次 CS 框住, 比逐字节翻转 CS 可靠 */
        oled_dc_cmd();
        oled_cs_low();

        oled_spi_write((u8)(0xB0+i));                                        //页地址
        oled_spi_write((u8)(0x00 | (g_oled_col_offset & 0x0FU)));            //低列地址(含偏移)
        oled_spi_write((u8)(0x10 | ((g_oled_col_offset >> 4) & 0x0FU)));     //高列地址

        oled_spi_flush();       /* ★ 等这 3 个命令真发完, 才能切 DC 到数据模式 */
        oled_dc_data();
        for(n=0;n<128;n++)
            oled_spi_write(OLED_GRAM[n][i]);

        oled_spi_flush();       /* 等这一页真正发完, 再放开片选 */
        oled_cs_high();
    }
}

void OLED_Clear(void)
{
    u8 i,n;
    for(i=0;i<8;i++)
        for(n=0;n<128;n++)
            OLED_GRAM[n][i]=0x00;
    OLED_Refresh();
}

void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2)
{
    u8 t;
    int xerr=0,yerr=0,delta_x,delta_y,distance;
    int incx,incy,uRow,uCol;
    delta_x=x2-x1;
    delta_y=y2-y1;
    uRow=x1;
    uCol=y1;
    if(delta_x>0)incx=1;
    else if(delta_x==0)incx=0;
    else {incx=-1;delta_x=-delta_x;}
    if(delta_y>0)incy=1;
    else if(delta_y==0)incy=0;
    else {incy=-1;delta_y=-delta_y;}
    if(delta_x>delta_y)distance=delta_x;
    else distance=delta_y;
    for(t=0;t<=distance+1;t++)
    {
        OLED_DrawPoint(uRow,uCol);
        xerr+=delta_x;
        yerr+=delta_y;
        if(xerr>distance) { xerr-=distance; uRow+=incx; }
        if(yerr>distance) { yerr-=distance; uCol+=incy; }
    }
}

void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
    int a=0,b=r,d;
    d = 1 - (int)r;
    while(a <= b)
    {
        OLED_DrawPoint(x+a,y+b); OLED_DrawPoint(x-a,y+b);
        OLED_DrawPoint(x+a,y-b); OLED_DrawPoint(x-a,y-b);
        OLED_DrawPoint(x+b,y+a); OLED_DrawPoint(x-b,y+a);
        OLED_DrawPoint(x+b,y-a); OLED_DrawPoint(x-b,y-a);
        if(d<0) { d += 2*a+3; }
        else    { d += 2*(a-b)+5; b--; }
        a++;
    }
}

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1)
{
    u8 i,m,temp,size2,chr1;
    u8 y0=y;
    size2=(size1/8+((size1%8)?1:0))*(size1/2);
    chr1=chr-' ';
    for(i=0;i<size2;i++)
    {
        if(size1==12)      temp=asc2_1206[chr1][i];
        else if(size1==16) temp=asc2_1608[chr1][i];
        else if(size1==24) temp=asc2_2412[chr1][i];
        else return;
        for(m=0;m<8;m++)
        {
            if(temp&0x80) OLED_DrawPoint(x,y);
            else          OLED_ClearPoint(x,y);
            temp<<=1;
            y++;
            if((y-y0)==size1) { y=y0; x++; break; }
        }
    }
}

void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1)
{
    while((*chr>=' ')&&(*chr<='~'))
    {
        OLED_ShowChar(x,y,*chr,size1);
        x+=size1/2;
        if(x>128-size1/2) { x=0; y+=size1; }
        chr++;
    }
}

static u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--) result*=m;
    return result;
}

void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1)
{
    u8 t,temp;
    u8 enshow=0;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0) { OLED_ShowChar(x+(size1/2)*t,y,' ',size1); continue; }
            else enshow=1;
        }
        OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
    }
}

void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1)
{
    u8 i,m,n=0,temp,chr1;
    u8 y0=y;
    u8 size3=size1/8;
    while(size3--)
    {
        chr1=num*size1/8+n;
        n++;
        for(i=0;i<size1;i++)
        {
            if(size1==16)      temp=Hzk1[chr1][i];
            else if(size1==24) temp=Hzk2[chr1][i];
            else if(size1==32) temp=Hzk3[chr1][i];
            else if(size1==64) temp=Hzk4[chr1][i];
            else return;
            for(m=0;m<8;m++)
            {
                if(temp&0x80) OLED_DrawPoint(x,y);
                else          OLED_ClearPoint(x,y);
                temp<<=1;
                y++;
                if((y-y0)==size1) { y=y0; x++; break; }
            }
        }
    }
}

/* 设置写入起始位置: x=列(0~127), y=页(0~7) */
void OLED_WR_BP(u8 x,u8 y)
{
    u8 col = (u8)(x + g_oled_col_offset);   /* SH1106 必须加偏移 */
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte((u8)(0x10|((col>>4)&0x0f)),OLED_CMD);
    OLED_WR_Byte((u8)(col&0x0f),OLED_CMD);
}

void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[])
{
    u32 j=0;
    u8 x=0,y=0;
    if(y%8==0)y=0;
    else y+=1;
    for(y=y0;y<y1;y++)
    {
        OLED_WR_BP(x0,y);
        for(x=x0;x<x1;x++)
        {
            OLED_WR_Byte(BMP[j],OLED_DATA);
            j++;
        }
    }
}

/* 背光控制(BLK 脚, 高电平点亮; 若你的模块是低电平点亮, 把 ON/OFF 里的宏对调) */
void OLED_BacklightOn(void)  { OLED_BLK_ON();  }
void OLED_BacklightOff(void) { OLED_BLK_OFF(); }

/* ============================================================================
 *  初始化: 硬件复位 + SSD1306 命令序列
 * ==========================================================================*/
void OLED_Init(void)
{
    /* 空闲状态: 片选拉高(不选中), DC 默认命令, 背光点亮 */
    OLED_CS_HIGH();
    OLED_DC_CMD();
    OLED_BLK_ON();

    /* 硬件复位: 高 -> 拉低 -> 拉高, 延时按商家例程给足(100/200ms) */
    OLED_RES_HIGH();
    delay_ms(100);
    OLED_RES_LOW();
    delay_ms(200);
    OLED_RES_HIGH();

    OLED_WR_Byte(0xAE,OLED_CMD);    //关显示
    OLED_WR_Byte((u8)(0x00|(g_oled_col_offset&0x0f)),OLED_CMD);   //低列地址(含偏移)
    OLED_WR_Byte((u8)(0x10|((g_oled_col_offset>>4)&0x0f)),OLED_CMD);//高列地址
    OLED_WR_Byte(0x40,OLED_CMD);    //起始行
    OLED_WR_Byte(0x81,OLED_CMD);    //对比度
    OLED_WR_Byte(0xCF,OLED_CMD);
    OLED_WR_Byte(OLED_SEG_REMAP,OLED_CMD);   //左右方向(0xA0/0xA1)
    OLED_WR_Byte(OLED_COM_SCAN,OLED_CMD);    //上下方向(0xC0/0xC8)
    OLED_WR_Byte(0xA6,OLED_CMD);    //正常显示
    OLED_WR_Byte(0xA8,OLED_CMD);    //多路复用比
    OLED_WR_Byte(0x3f,OLED_CMD);    //1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);    //显示偏移
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0xd5,OLED_CMD);    //时钟分频
    OLED_WR_Byte(0x80,OLED_CMD);
    OLED_WR_Byte(0xD9,OLED_CMD);    //预充电周期
    OLED_WR_Byte(0xF1,OLED_CMD);
    OLED_WR_Byte(0xDA,OLED_CMD);    //COM 引脚配置
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);    //VCOMH
    OLED_WR_Byte(0x40,OLED_CMD);
    OLED_WR_Byte(0x20,OLED_CMD);    //寻址模式: 页寻址(商家例程同款)
    OLED_WR_Byte(0x02,OLED_CMD);
    OLED_WR_Byte(0x8D,OLED_CMD);    //电荷泵使能(商家例程同款)
    OLED_WR_Byte(0x14,OLED_CMD);
    OLED_WR_Byte(0xA4,OLED_CMD);    //全局显示跟随显存
    OLED_WR_Byte(0xA6,OLED_CMD);    //非反色
    OLED_WR_Byte(0xAF,OLED_CMD);    //开显示

    OLED_Clear();
}
