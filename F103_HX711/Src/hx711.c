#include "hx711.h"
#include "main.h"
void HX711_Init(HX711 data)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = data.pinSck;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioSck, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = data.pinData;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioData, &GPIO_InitStruct);

	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
}

int averageValue(HX711 data, uint8_t times)
{
    int sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += getValue(data);
    }
    return sum / times;
}

int getValue(HX711 data)
{
    int buffer;
    buffer = 0;

    while (HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1);

    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(data.gpioData, data.pinData))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }

    for (int i = 0; i < data.gain; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }

    buffer = buffer ^ 0x800000;

    return buffer;
}

HX711 setTare(HX711 data, uint8_t times)
{
    int sum = averageValue(data, times);
    data.offset = sum;
    return data;
}

float getGram(HX711 data, uint8_t times)
{
    long val = averageValue(data, times) - data.offset;
    return (float) val / data.scale;
}


void powerDown(HX711 data){}
void powerUp(HX711 data){}

