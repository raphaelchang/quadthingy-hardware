#ifndef ESC_H_
#define ESC_H_

void esc_init(void);
void esc_set(uint8_t id, double duty_cycle);
void esc_set_all(double duty_cycle);

#endif /* ESC_H_ */