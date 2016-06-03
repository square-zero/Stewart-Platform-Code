#ifndef ps2_h
#define ps2_h

#include <Arduino.h>

class PS2 {
  public:
    PS2(int clk, int data);
    void write(unsigned char data);
    unsigned char read(void);
    
    void mouse_init(void);
    void mouse_pos(char &stat, char &x, char &y);
    
  private:
    int _ps2clk;
    int _ps2data;
    void golo(int pin);
    void gohi(int pin);
};

#endif
