#include "PixyCam.h"
#include <Arduino.h>

PixyCam::PixyCam() {}

void PixyCam::init() {
  Serial.println("Starting Pixy2...");
  pixy.init();
}

int32_t PixyCam::getXOffset(uint8_t signature) {
  pixy.ccc.getBlocks(false, signature); // 指定されたシグネチャのみを要求
  if (pixy.ccc.numBlocks) {
    // シグネチャに一致するブロックの中で最も大きいものを追跡対象とする
    int largestBlock = -1;
    int largestSize = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == signature) {
        int currentSize = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        if (currentSize > largestSize) {
          largestSize = currentSize;
          largestBlock = i;
        }
      }
    }
    if (largestBlock != -1) {
      return (int32_t)pixy.ccc.blocks[largestBlock].m_x - (320 / 2);
    }
  }
  return -1; // 指定されたシグネチャのブロックが検出されなかった場合
}

