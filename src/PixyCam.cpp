#include "PixyCam.h"
#include <Arduino.h>

PixyCam::PixyCam() {}

void PixyCam::init() {
  Serial.println("Initializing Pixy2...");
  pixy.init();
  // Per user feedback, PIXY2 settings are pre-configured. Do not set them in code.
  // Use default lamp settings: Upper LEDs on, lower off
  pixy.setLamp(1, 0);
}

bool PixyCam::getBestBlock(uint8_t signature, Block& foundBlock) {
  // Get all detected blocks from Pixy2
  pixy.ccc.getBlocks(false, signature);

  if (pixy.ccc.numBlocks == 0) {
    return false; // No blocks detected at all
  }

  int largestArea = 0;
  int bestBlockIndex = -1;

  // The vertical midpoint of the screen
  const int screenMidY = PIXY_FRAME_HEIGHT / 2;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    // Check if the block matches the desired signature and is in the lower half of the screen
    if (pixy.ccc.blocks[i].m_signature == signature && pixy.ccc.blocks[i].m_y > screenMidY) {
      int currentArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      if (currentArea > largestArea) {
        largestArea = currentArea;
        bestBlockIndex = i;
      }
    }
  }

  // If a valid block was found
  if (bestBlockIndex != -1) {
    // Assign the found block data to the block passed by reference.
    // This copies the entire struct.
    foundBlock = pixy.ccc.blocks[bestBlockIndex];
    return true;
  }

  // No block met the criteria
  return false;
}
