#include "stm32f4xx_hal.h"

class GPIO{
public:
	GPIO(GPIO_TypeDef* Port, uint16_t Pin);
	void Set();
	void Reset();
	bool Status();
private:
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;
};
