#include "PixyCam.h"
#include <Arduino.h>

PixyCam::PixyCam() {}

void PixyCam::init() {
  Serial.println("Initializing Pixy2...");
  pixy.init();
  // Set camera brightness and saturation for better color detection
  pixy.setCameraBrightness(80);
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
    // Populate the Block struct with the data from the best block
    foundBlock.signature = pixy.ccc.blocks[bestBlockIndex].m_signature;
    foundBlock.x = pixy.ccc.blocks[bestBlockIndex].m_x;
    foundBlock.y = pixy.ccc.blocks[bestBlockIndex].m_y;
    foundBlock.width = pixy.ccc.blocks[bestBlockIndex].m_width;
    foundBlock.height = pixy.ccc.blocks[bestBlockIndex].m_height;
    foundBlock.age = pixy.ccc.blocks[bestBlockIndex].m_age;
    foundBlock.index = pixy.ccc.blocks[bestBlockIndex].m_index;
    return true;
  }

  // No block met the criteria
  return false;
}
