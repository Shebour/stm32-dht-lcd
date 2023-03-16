#include "dht.h"
#include "main.h"

uint32_t pMillis, cMillis;

uint8_t DHT11_Start(void) {
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT_Pin;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT_GPIO_Port,
                &GPIO_InitStructPrivate);       // set the pin as output
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 0); // pull the pin low
  HAL_Delay(20);                                // wait for 20ms
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 1);
  // pull the pin high
  microDelay(30); // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStructPrivate); // set the pin as input
  microDelay(40);
  if (!(HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin))) {
    microDelay(80);
    if ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
      Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) && pMillis + 2 > cMillis) {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read(void) {
  uint8_t a, b;
  b = 0;
  for (a = 0; a < 8; a++) {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) &&
           pMillis + 2 > cMillis) { // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay(40);                                  // wait for 40 us
    if (!(HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin))) // if the pin is low
      b &= ~(1 << (7 - a));
    else
      b |= (1 << (7 - a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) &&
           pMillis + 2 > cMillis) { // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

int DHT_Read_A(struct DHT_data *dht_data) {
  if (!DHT11_Start()) {
    return -1;
  }
  uint8_t RHI = DHT11_Read();
  uint8_t RHD = DHT11_Read();
  uint8_t TCI = DHT11_Read();
  uint8_t TCD = DHT11_Read();
  uint8_t SUM = DHT11_Read();
  if (RHI + RHD + TCI + TCD == SUM) {
    dht_data->humidity = (float)RHI + (float)(RHD / 10.0);
    dht_data->temperature = (float)TCI + (float)(TCD / 10.0);
    return 1;
  }
  return 0;
}
