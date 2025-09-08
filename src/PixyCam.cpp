#include "PixyCam.h"
#include <Arduino.h>

PixyCam::PixyCam() : m_lastTrackedIndex(-1) {}

void PixyCam::init() {
  Serial.println("Initializing Pixy2...");
  pixy.init();
  pixy.setLamp(1, 0);
}

void PixyCam::resetTracking() {
  m_lastTrackedIndex = -1;
}

bool PixyCam::getBestBlock(uint8_t signature, Block& foundBlock) {
  pixy.ccc.getBlocks(false, signature);

  if (pixy.ccc.numBlocks == 0) {
    resetTracking(); // No blocks visible, so reset tracking
    return false;
  }

  // --- Sticky Target Logic ---
  // 1. If we were already tracking a block, try to find it again.
  if (m_lastTrackedIndex != -1) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_index == m_lastTrackedIndex) {
        // Found our old block. Update and return.
        foundBlock = pixy.ccc.blocks[i];
        return true;
      }
    }
    // If we get here, our tracked block was lost.
    resetTracking();
  }

  // 2. If not tracking, or if the tracked block was lost, find the new best block.
  // The best block is the largest one in the lower half of the screen.
  int largestArea = 0;
  int bestBlockIndex = -1;
  const int screenMidY = PIXY_FRAME_HEIGHT / 2;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_y > screenMidY) {
      int currentArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      if (currentArea > largestArea) {
        largestArea = currentArea;
        bestBlockIndex = i;
      }
    }
  }

  if (bestBlockIndex != -1) {
    foundBlock = pixy.ccc.blocks[bestBlockIndex];
    m_lastTrackedIndex = foundBlock.m_index; // Lock onto this new block
    return true;
  }

  // No valid blocks found in the lower half of the screen
  return false;
}
