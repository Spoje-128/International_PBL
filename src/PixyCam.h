#ifndef PIXY_CAM_H
#define PIXY_CAM_H

#include <Pixy2.h>

// A struct to hold information about a detected block
struct Block {
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  int16_t  age; // How many frames the block has been tracked
  int16_t  index; // The block's index in the pixy.ccc.blocks[] array
};

class PixyCam {
public:
  PixyCam();
  void init();

  // Finds the best block based on signature and position criteria.
  // Returns true if a valid block is found and populates the 'block' argument.
  // Returns false if no valid block is found.
  bool getBestBlock(uint8_t signature, Block& block);

private:
  Pixy2 pixy;

  // Pixy2 camera resolution constants
  static const int PIXY_FRAME_WIDTH = 316;
  static const int PIXY_FRAME_HEIGHT = 208;
};

#endif // PIXY_CAM_H
