#ifndef __W1_H__
#define __W1_H__

#define W1_PIN 0	/* Pin 0 on Port B */

uint8_t w1_crc(uint8_t *buf, uint8_t len);
void w1_write(uint8_t val);
uint8_t w1_read_byte();
void w1_read(uint8_t *buf, uint8_t len);
bool w1_reset(void);
void w1_setup(void);

#endif /* __W1_H__ */
