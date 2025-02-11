#ifndef HX711_H_
#define HX711_H_

#include "main.h"


typedef struct _hx711
{
	GPIO_TypeDef* gpioSck;
	GPIO_TypeDef* gpioData;
	uint16_t pinSck;
	uint16_t pinData;
	int offset;
	int gain;
	float scale;
	// 1: channel A, gain factor 128
	// 2: channel B, gain factor 32
    // 3: channel A, gain factor 64
} HX711;

void  HX711_Init(HX711 data);
int averageValue(HX711 data, uint8_t times);
int getValue(HX711 data);
HX711 setTare(HX711 data, uint8_t times);

float getGram(HX711 data, uint8_t times);






//HX711 HX711_Tare(HX711 data, uint8_t times);
//int   HX711_Value(HX711 data);
//int   HX711_AverageValue(HX711 data, uint8_t times);
//HX711 setTare(HX711 data, uint8_t times)
//int   getValue(HX711 data);
//int   averageValue(HX711 data, uint8_t times);
//void  setOffset(HX711 data);
//void  setScale(HX711 data);
//float getGram(HX711 data, uint8_t times);
//void  setGain(HX711 data, uint8_t gain);
//void  powerDown(HX711 data);
//void  powerUp(HX711 data);
//void  tare(HX711 data, uint8_t times);


#endif /* HX711_H_ */
