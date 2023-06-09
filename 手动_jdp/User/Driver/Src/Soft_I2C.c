/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                开源并不等于免费，先驱者的历史已经证明，在国内这个环境下，毫无收益的开源，单靠坊间个人爱好者，自发地参与项
								目完善的方式行不通，好的开源项目需要请专职人员做好售后服务、手把手教学、统计用户反馈需求、在实践中完成对产
								品的一次次完善与迭代升级。经过综合考虑，无名飞控开源代码中，程序仅保留公司正版激活功能代码，版本激活无实际
								功能，属于公司产品常规出厂操作，不影响客户学习，其余代码全部开放给客户学习，客户移植和二次开发代码请保留代
								码版权。
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：2号群465082224、1号群540707961（人员已满）
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               修改日期:2021/04/30                    
*               版本：筑梦者PLUS——GankerPilot_V3.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2025 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Soft_I2C.h"


#define GPIO_I2C   		GPIO_PORT_P10 
#define SCL_PIN   		GPIO_PIN3
#define SDA_PIN   		GPIO_PIN2
#define I2C_READ_SDA  GPIO_getInputPinValue(GPIO_I2C,SDA_PIN)			//SDA 
#define I2C_SDA_H  		GPIO_setOutputHighOnPin(GPIO_I2C,SDA_PIN)//SDA 
#define I2C_SDA_L 		GPIO_setOutputLowOnPin(GPIO_I2C,SDA_PIN)			//SDA
#define I2C_SCL_H  		GPIO_setOutputHighOnPin(GPIO_I2C,SCL_PIN)//SCL
#define I2C_SCL_L  		GPIO_setOutputLowOnPin(GPIO_I2C,SCL_PIN)			//SCL


void I2C_GPIO_Config(void)
{
	MAP_GPIO_setAsOutputPin(GPIO_I2C, SCL_PIN);
	MAP_GPIO_setAsOutputPin(GPIO_I2C, SDA_PIN);
	
  Delay_Ms(10);
	MAP_GPIO_setOutputHighOnPin(GPIO_I2C,SCL_PIN);
	MAP_GPIO_setOutputHighOnPin(GPIO_I2C,SDA_PIN);
}


void I2C_SDA_OUT(void)
{
	GPIO_setAsOutputPin(GPIO_I2C,SDA_PIN);//SDA
}

void I2C_SDA_IN(void)
{
  GPIO_setAsInputPin(GPIO_I2C,SDA_PIN);//SDA
}

#define I2C_SDA_OUT I2C_SDA_OUT()
#define I2C_SDA_IN  I2C_SDA_IN()
#define I2C_sdaRead I2C_READ_SDA  		


static void i2cDelay()
{
//    volatile int i = 7;//7
//    while (i)
//    i--;
}
 
// SCL高电平期间，SDA出现下降沿为起始信号
static bool i2cStart()
{
    I2C_SDA_OUT;
    I2C_SCL_H;
    I2C_SDA_H;
    i2cDelay();
    if (!I2C_sdaRead)  // 如果SDA为低电平，则总线忙，退出
        return false;
    I2C_SDA_L;
    if (I2C_sdaRead)  // 如果SDA为高电平，则总线忙，退出
        return false;
    I2C_SDA_L;
    return true;
}
 
// SCL高电平期间，SDA出现上升沿为停止信号
static void i2cStop(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L; 
    I2C_SDA_L;
    i2cDelay();  // STOP:when CLK is high DATA from low to high 
    I2C_SCL_H;
    I2C_SDA_H;  
    i2cDelay();
}
 
static void i2cAck(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L;
    i2cDelay();
    I2C_SDA_L;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    I2C_SCL_L;
}
 
static void i2cNoAck(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L;
    i2cDelay();
    I2C_SDA_H;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    I2C_SCL_L;
}
 
// SCL高电平期间，SDA电平被从设备拉低表示应答
static bool i2cWaitAck(void)
{
    uint8_t errTimes = 0;
    I2C_SDA_IN;
    I2C_SDA_H;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    while (I2C_sdaRead) {
        if (errTimes++ > 20) {
            I2C_SCL_L;
            return false;
        }           
        i2cDelay();
    }
    I2C_SCL_L;
    return true;
}
 
// 发送数据，数据从高位到低位传输  
static void i2cSendByte(uint8_t byte)  
{
    uint8_t i = 8;
 
    I2C_SDA_OUT;
    while (i--) {      
        I2C_SCL_L;  // 时钟信号为低电平期间，允许数据线电平变化
        i2cDelay();
        if (byte & 0x80)
            I2C_SDA_H;
        else
            I2C_SDA_L; 
        byte <<= 1; 
        i2cDelay();
        I2C_SCL_H;
        i2cDelay();
    }
    I2C_SCL_L;
}
 
static uint8_t i2cReceiveByte()  
{
    uint8_t i = 8;
    uint8_t byte = 0;
    I2C_SDA_IN;
    I2C_SDA_H;
    while (i--) {
        byte <<= 1;
        I2C_SCL_H;
        i2cDelay();
        if (I2C_sdaRead) {
            byte |= 0x01;
        }
        I2C_SCL_L;
        i2cDelay();
    }
    I2C_SCL_L;
    return byte; 
}
 
 

 
/**
 * 通过I2C总线写一字节数据
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] data:要写入的数据
 */
bool i2cWriteOneByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);  // 从机地址由高7位+读写位构成   
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);       
    i2cWaitAck();
    i2cSendByte(data);     
    i2cWaitAck();
    return true;
}
 

/**
 *  
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:字节数 
 * @param[in] data:待写入的数据 
 */
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg)
{
	unsigned char REG_data;
  i2cStart();
  i2cSendByte(dev<<1);
	i2cWaitAck();
  i2cSendByte(reg);
	i2cWaitAck();
  i2cStart();
  i2cSendByte((dev << 1) | 0x01);
  i2cWaitAck();
	REG_data=i2cReceiveByte();
  i2cStop();
  return REG_data;	
} 


/**
 *  
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:字节数 
 * @param[in] data:待写入的数据 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;
 
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);          
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(dev);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}
 
 
/**
 * 从I2C设备中读取数据
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:数据字节数
 * @param[out] data:读出的数据
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);      
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // 器件地址+读命令    
    i2cWaitAck();
    while (len) {
        *data = i2cReceiveByte();
        if (len == 1)
            i2cNoAck();  // 最后一个字节不应答
        else
            i2cAck();
        data++;
        len--;
    }
    i2cStop();
    return true;
}

void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
  i2cWriteOneByte(SlaveAddress,REG_Address,REG_data);
}

//**************************************
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address)
{
  return i2cReadOneBytes(SlaveAddress,REG_Address);
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

