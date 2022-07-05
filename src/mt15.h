#ifndef _MT15_H
#define _MT15_H

class MT15Class 
{
  private:
    String sendCommand(String command);
  
  public:
    void begin(int baud);
};

extern MT15Class MT15;

#endif
