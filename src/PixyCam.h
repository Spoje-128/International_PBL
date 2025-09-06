#ifndef PIXY_CAM_H
#define PIXY_CAM_H

#include <Pixy2.h>

class PixyCam {
public:
  PixyCam();
  void init();
  int32_t getXOffset(uint8_t signature);

private:
  Pixy2 pixy;
};

#endif // PIXY_CAM_H
