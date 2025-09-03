#include "PixyCam.h"
#include <Arduino.h>

PixyCam::PixyCam() {}

void PixyCam::init() {
  Serial.println("Starting Pixy2...");
  pixy.init();
}

int32_t PixyCam::getXOffset() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    // 最も大きいブロックを追跡対象とする
    int largestBlock = -1;
    int largestSize = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int currentSize = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      if (currentSize > largestSize) {
        largestSize = currentSize;
        largestBlock = i;
      }
    }
    return (int32_t)pixy.ccc.blocks[largestBlock].m_x - (320 / 2);
  }
  return -1; // ブロックが検出されなかった場合
}

