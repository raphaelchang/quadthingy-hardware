#ifndef BNO055_H_
#define BNO055_H_

bool bno055_init(void);
bool bno055_init(void);
uint8_t bno055_read_addr(uint8_t addr);
void bno055_write_addr(uint8_t addr, uint8_t value);

#endif /* #define BNO055_H_ */