#pragma once

#include "Runtime/CIOWin.hpp"
#include "Runtime/CToken.hpp"

namespace urde {
class CStringTable;
class CRasterFont;
class CMoviePlayer;
struct CFinalInput;

namespace MP1 {
class CCredits : public CIOWin {
  u32 x14_ = 0;
  TLockedToken<CStringTable> x18_creditsTable;
  TLockedToken<CRasterFont> x20_creditsFont;
  std::unique_ptr<CMoviePlayer> x28_;
  u32 x2c_ = 0;
  /* x34_  I think this is a vector */
  u32 x44_ = 0;
  float x48_ = 0.f;
  float x4c_ = 0.f;
  float x50_ = 32.f;
  float x54_;
  float x58_ = 0.f;
  bool x5c_24_ : 1;
  bool x5c_25_ : 1;
  bool x5c_26_ : 1;
  bool x5c_27_ : 1;
  bool x5c_28_ : 1;

public:
  CCredits();
  EMessageReturn OnMessage(const CArchitectureMessage&, CArchitectureQueue&) override;
  bool GetIsContinueDraw() const override { return false; }
  void Draw() const override;

  void Update(float, CArchitectureQueue& queue);
  void ProcessUserInput(const CFinalInput& input);
};
} // namespace MP1
} // namespace urde
