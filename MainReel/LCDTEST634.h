#ifndef LCDTEST634_H_
#define LCDTEST634_H_

void encoder_isr(uint gpio, uint32_t events);
void screen_setup();
void screen_update(int linelength, int dragset)

#endif