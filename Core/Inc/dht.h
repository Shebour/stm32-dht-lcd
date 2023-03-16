#ifndef DHT_H
#define DHT_H

struct DHT_data {
  float temperature;
  float humidity;
};

int DHT_Read_A(struct DHT_data *dht_data);

#endif /* DHT_H */
