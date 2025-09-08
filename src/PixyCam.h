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

  // Finds the best block using "sticky" logic to avoid target switching.
  bool getBestBlock(uint8_t signature, Block& block);

  // Resets the tracking logic to look for a new target.
  void resetTracking();

private:
  Pixy2 pixy;
  int m_lastTrackedIndex; // Stores the index of the last block we were tracking
};

#endif // PIXY_CAM_H
