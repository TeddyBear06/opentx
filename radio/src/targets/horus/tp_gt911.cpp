/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "tp_gt911.h"

uint8_t touchGT911Flag = 0;
uint8_t touchPanelEvent = 0;
struct TouchData touchData;
struct TouchState touchState;

static void TOUCH_AF_ExtiStop(void)
{
  SYSCFG_EXTILineConfig(TOUCH_INT_EXTI_PortSource, TOUCH_INT_EXTI_PinSource1);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = TOUCH_INT_EXTI_LINE1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);


  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TOUCH_INT_EXTI_IRQn1;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void TOUCH_AF_ExtiConfig(void)
{
  SYSCFG_EXTILineConfig(TOUCH_INT_EXTI_PortSource, TOUCH_INT_EXTI_PinSource1);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = TOUCH_INT_EXTI_LINE1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TOUCH_INT_EXTI_IRQn1;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void I2C_FreeBus()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // reset i2c bus by setting clk as output and sending manual clock pulses
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

  //send 100khz clock train for some 100ms
  tmr10ms_t until = get_tmr10ms() + 11;
  while (get_tmr10ms() < until) {
    if (GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA_GPIO_PIN) == 1) {
      TRACE("touch: i2c free again\n");
      break;
    }
    TRACE("i2c bus freed");
    I2C_GPIO->BSRRH = I2C_SCL_GPIO_PIN;
    delay_us(10);
    I2C_GPIO->BSRRL = I2C_SCL_GPIO_PIN;
    delay_us(10);
  }
  //send stop condition:
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

  //clock is low
  I2C_GPIO->BSRRH = I2C_SCL_GPIO_PIN;
  delay_us(10);
  //sda = lo
  I2C_GPIO->BSRRL = I2C_SDA_GPIO_PIN;
  delay_us(10);
  //clock goes high
  I2C_GPIO->BSRRH = I2C_SCL_GPIO_PIN;
  delay_us(10);
  //sda = hi
  I2C_GPIO->BSRRL = I2C_SDA_GPIO_PIN;
  delay_us(10);
  TRACE("FREE BUS");
}

void Touch_DeInit()
{
  I2C_DeInit(I2C);
  (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST));
  delay_ms(150);
  (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST));
}

void I2C_Init()
{
  Touch_DeInit();

  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

  I2C_FreeBus();

  GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource7, GPIO_AF_I2C1);
  GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource8, GPIO_AF_I2C1);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN | I2C_SDA_GPIO_PIN;
  GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = TOUCH_RST_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL; //
  GPIO_Init(TOUCH_RST_GPIO, &GPIO_InitStructure);


  //https://community.st.com/s/question/0D50X00009XkZ9FSAV/stm32f4-i2c-issues-solved
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = 400000;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C, &I2C_InitStructure);

  I2C_Init(I2C, &I2C_InitStructure);
  //I2C_StretchClockCmd(I2C, ENABLE);
  I2C_Cmd(I2C, ENABLE);

  //ext interupt
  GPIO_InitStructure.GPIO_Pin = TOUCH_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TOUCH_INT_GPIO, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

#define I2C_TIMEOUT_MAX 1000

bool I2C_WaitEvent(uint32_t event)
{
  uint32_t timeout = I2C_TIMEOUT_MAX;
  while (!I2C_CheckEvent(I2C, event)) {
    if ((timeout--) == 0) return false;
  }
  return true;
}

bool I2C_WaitEventCleared(uint32_t event)
{
  uint32_t timeout = I2C_TIMEOUT_MAX;
  while (I2C_CheckEvent(I2C, event)) {
    if ((timeout--) == 0) return false;
  }
  return true;
}

bool I2C_Send7BitAddress(uint8_t address, uint16_t direction)
{
  I2C_SendData(I2C, (address << 1) | ((direction == I2C_Direction_Receiver) ? 1 : 0));
  // check if slave acknowledged his address within timeout
  if (!I2C_WaitEvent(direction == I2C_Direction_Transmitter ? I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED : I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    return false;
  return true;
}

bool touch_i2c_read(uint8_t addr, uint8_t reg, uint8_t * data, uint8_t len)
{
  if (!I2C_WaitEventCleared(I2C_FLAG_BUSY)) return false;
  I2C_GenerateSTART(I2C, ENABLE);
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false;
  if (!I2C_Send7BitAddress(addr, I2C_Direction_Transmitter)) return false;
  I2C_SendData(I2C, reg);
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return false;
  I2C_GenerateSTART(I2C, ENABLE);
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false;
  if (!I2C_Send7BitAddress(addr, I2C_Direction_Receiver)) return false;

  if (len > 1) I2C_AcknowledgeConfig(I2C, ENABLE);

  while (len) {
    if (len == 1) I2C_AcknowledgeConfig(I2C, DISABLE);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))return false;
    *data++ = I2C_ReceiveData(I2C);;
    len--;
  }
  I2C_GenerateSTOP(I2C, ENABLE);
  return true;
}

static bool touch_i2c_write(uint8_t addr, uint8_t reg, uint8_t * data, uint8_t len)
{
  if (!I2C_WaitEventCleared(I2C_FLAG_BUSY)) return false;
  I2C_GenerateSTART(I2C, ENABLE);
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return false;

  if (!I2C_Send7BitAddress(addr, I2C_Direction_Transmitter)) return false;
  I2C_SendData(I2C, (uint8_t) ((reg & 0xFF00) >> 8));
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) return false;
  I2C_SendData(I2C, (uint8_t) (reg & 0x00FF));
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) return false;
  while (len--) {
    I2C_SendData(I2C, *data);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) return false;
    data++;
  }
  if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return false;
  I2C_GenerateSTOP(I2C, ENABLE);
  return true;
}

void GT911_INT_Change(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = TOUCH_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(TOUCH_INT_GPIO, &GPIO_InitStructure);
}

uint8_t GT911_Send_Cfg(uint8_t mode)
{
  uint8_t buf[2];
  uint8_t i = 0;
  buf[0] = 0;
  buf[1] = mode;
  for (i = 0; i < sizeof(GT911_Cfg); i++)
    buf[0] += GT911_Cfg[i];//check sum

  buf[0] = (~buf[0]) + 1;
  gt911WriteRegister(GT_CFGS_REG, (uint8_t *) GT911_Cfg, sizeof(GT911_Cfg));
  gt911WriteRegister(GT_CHECK_REG, buf, 2); //write checksum
  return 0;
}

uint8_t gt911WriteRegister(uint16_t reg, uint8_t * buffer, uint8_t length)
{
  uint8_t tryCount = 3;

  I2C_SendData(I2C, GT_CMD_WR);     //send cmd
  I2C_SendData(I2C, reg >> 8);      //send hi
  I2C_SendData(I2C, reg & 0XFF);    //send low
  while (!touch_i2c_write(TOUCH_I2C_ADDRESS, reg, buffer, length)) {
    if (--tryCount == 0) break;
    I2C_Init();
  }
  return 1;
}


uint8_t gt911ReadRegister(uint16_t reg, uint8_t * buffer, uint8_t length)
{
  uint8_t tryCount = 3;
  I2C_SendData(I2C, GT_CMD_WR);     //send cmd
  I2C_SendData(I2C, reg >> 8);      //send hi
  I2C_SendData(I2C, reg & 0XFF);    //send low
  I2C_SendData(I2C, GT_CMD_RD);
  while (!touch_i2c_read(TOUCH_I2C_ADDRESS, reg, buffer, length)) {
    if (--tryCount == 0) break;
    I2C_Init();
  }
  return 1;
}

void TouchReset()
{
  GPIO_ResetBits(TOUCH_RST_GPIO, TOUCH_RST_GPIO_PIN);
  delay_ms(20);
  GPIO_SetBits(TOUCH_RST_GPIO, TOUCH_RST_GPIO_PIN);
  delay_ms(300);
}

void touchPanelDeInit(void)
{
  TOUCH_AF_ExtiStop();
}


bool touchPanelInit(void)
{
  uint8_t tmp[4] = {0};

  TRACE("Touchpanel init start ...");
  I2C_Init();

  TPRST_LOW();
  TPINT_HIGH();
  delay_us(200);

  TPRST_HIGH();
  delay_ms(6);

  TPINT_LOW();
  delay_ms(55);

  GT911_INT_Change();  //Set INT INPUT INT=LOW

  delay_ms(50);

  TRACE("Reading Touch registry");
  gt911ReadRegister(GT_PID_REG, tmp, 4);

  if (strcmp((char *) tmp, "911") == 0) //ID==9147
  {
    TRACE("GT911 chip detected");
    tmp[0] = 0X02;
    gt911WriteRegister(GT_CTRL_REG, tmp, 1);
    gt911ReadRegister(GT_CFGS_REG, tmp, 1);

    TRACE("Chip config Ver:%x\r\n",tmp[0]);
    if (tmp[0] < GT911_CFG_NUMER)  //Config ver
    {
      TRACE("Sending new config %d", GT911_CFG_NUMER);
      GT911_Send_Cfg(1);
    }

    delay_ms(10);
    tmp[0] = 0X00;
    gt911WriteRegister(GT_CTRL_REG, tmp, 1);  //end reset
    touchGT911Flag = true;

    TOUCH_AF_ExtiConfig();

    return true;
  }
  TRACE("GT911 chip NOT FOUND");
  return false;
}

void touchPanelRead()
{
  uint8_t state = 0;
  gt911ReadRegister(GT911_READ_XY_REG, &state, 1);

  touchPanelEvent = false;

  if ((state & 0x80u) == 0x00) {
    // not ready
    return;
  }

  uint8_t pointsCount = (state & 0x0Fu);

  if (pointsCount > 0 && pointsCount < GT911_MAX_TP) {
    gt911ReadRegister(GT911_READ_XY_REG + 1, touchData.data, pointsCount * sizeof(TouchPoint));
    if (touchData.pointsCount == 0) {
      touchState.event = TE_DOWN;
      touchState.startX = touchState.x = touchData.points[0].x;
      touchState.startY = touchState.y = touchData.points[0].y;
    }
    else {
      touchState.deltaX = touchData.points[0].x - touchState.x;
      touchState.deltaY = touchData.points[0].y - touchState.y;
      if (touchState.event == TE_SLIDE || abs(touchState.deltaX) >= SLIDE_RANGE || abs(touchState.deltaY) >= SLIDE_RANGE) {
        touchState.event = TE_SLIDE;
        touchState.x = touchData.points[0].x;
        touchState.y = touchData.points[0].y;
      }
    }
    touchData.pointsCount = pointsCount;
  }
  else {
    if (touchData.pointsCount > 0) {
      touchData.pointsCount = 0;
      if (touchState.event == TE_SLIDE)
        touchState.event = TE_SLIDE_END;
      else
        touchState.event = TE_UP;
    }
  }

  uint8_t zero = 0;
  gt911WriteRegister(GT911_READ_XY_REG, &zero, 1);
}

extern "C" void TOUCH_INT_EXTI_IRQHandler1(void)
{
  if (EXTI_GetITStatus(TOUCH_INT_EXTI_LINE1) != RESET) {
    TRACE("TI");
    touchPanelEvent = 1;
    EXTI_ClearITPendingBit(TOUCH_INT_EXTI_LINE1);
  }
}
