#include "GPIO.h"

GPIO::GPIO(GPIO_TypeDef* Port, uint16_t Pin) {
	/*if(Port == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	if(Port == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	if(Port == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	if(Port == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	if(Port == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_Init(Port,&GPIO_InitStructure);*/


	if (Port == GPIOA)
		  __HAL_RCC_GPIOA_CLK_ENABLE();
	if (Port == GPIOB)
		  __HAL_RCC_GPIOB_CLK_ENABLE();
	if (Port == GPIOC)
		  __HAL_RCC_GPIOC_CLK_ENABLE();
	if (Port == GPIOD)
		  __HAL_RCC_GPIOD_CLK_ENABLE();
	if (Port == GPIOE)
		  __HAL_RCC_GPIOE_CLK_ENABLE();


	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	GPIO_Port = Port;
	GPIO_Pin = Pin;
}
bool GPIO::Status(){
  uint8_t status_bit = HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin);
  if(status_bit==GPIO_PIN_SET)
    return true;
  return false;
}
void GPIO::Set(){
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
	return;
}
void GPIO::Reset(){
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
	return;
}

