
#include "driver/i2c.h"
#include "paj7620.h"
#include "pag7660.h"
#include <string.h>

class Pixart_Gesture
{
   protected:
      uint8_t i2c_addr;
      i2c_port_t i2c_port = I2C_NUM_0;
      void writeReg(uint8_t addr, uint8_t value);
      void readRegs(uint8_t addr, uint8_t *values, int size);
      uint8_t readReg(uint8_t addr);
};


class paj7620 : public Pixart_Gesture
{
   public:
      paj7620();
      bool init();
      bool getResult(paj7620_gesture_t& res);

   private:
      bool setReportMode(uint8_t reportMode);
};


class pag7660 : public Pixart_Gesture
{
   public:
      pag7660(int mode = GESTURE_COMBINED_MODE) {
         gestureMode = mode;
      };
      bool init();
      bool getResult(pag7660_gesture_t& res);
      bool getOutput(pag7660_out_t& out);

      int getGestureMode();
      int nextGestureMode();

   private:
      uint8_t gestureMode;
      pag7660_out_t regToOutput(const pag7660_reg_out_t& reg);
};