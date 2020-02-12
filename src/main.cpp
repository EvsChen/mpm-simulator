#include "engine.h"

int main() {
  Engine engine;
  Float time = 0.f;
  for (int i = 0; i < 10; i++) {
    engine.P2GTransfer();
  }
}