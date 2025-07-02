#pragma once

class BLEUI {
public:
  BLEUI() = default;
  void begin();
  bool calibrationRequested();
  void handle();

private:
};
