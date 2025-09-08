#ifndef PIXY_CAM_H
#define PIXY_CAM_H

#include <Pixy2.h>

// NOTE: The 'Block' struct is defined within the Pixy2 library (Pixy2CCC.h).
// We will use that definition directly.

class PixyCam {
public:
  // Public constants for access from main.cpp
  static const int PIXY_FRAME_WIDTH = 316;
  static const int PIXY_FRAME_HEIGHT = 208;

  PixyCam();
  void init();

  // Finds the best block based on signature and position criteria.
  // Returns true if a valid block is found and populates the 'block' argument.
  // Returns false if no valid block is found.
  bool getBestBlock(uint8_t signature, Block& block);

private:
  Pixy2 pixy;
};

#endif // PIXY_CAM_H
