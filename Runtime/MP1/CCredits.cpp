#include "Runtime/MP1/CCredits.hpp"
#include "Runtime/CArchitectureMessage.hpp"
#include "Runtime/CArchitectureQueue.hpp"
#include "Runtime/Graphics/CGraphics.hpp"
#include "Runtime/Graphics/CMoviePlayer.hpp"
#include "Runtime/GuiSys/CRasterFont.hpp"
#include "Runtime/GuiSys/CStringTable.hpp"
#include "Runtime/Input/CFinalInput.hpp"
#include "Runtime/GameGlobalObjects.hpp"
#include "Runtime/CSimplePool.hpp"

namespace urde::MP1 {

CCredits::CCredits()
: CIOWin("Credits")
, x18_creditsTable(g_SimplePool->GetObj(g_tweakGui->GetCreditsTable()))
, x20_creditsFont(g_SimplePool->GetObj(g_tweakGui->GetCreditsFont()))
, x54_(g_tweakGui->x30c_)
, x5c_24_(false)
, x5c_25_(false)
, x5c_26_(false)
, x5c_27_(true)
, x5c_28_(false) {}

CIOWin::EMessageReturn CCredits::OnMessage(const CArchitectureMessage& msg, CArchitectureQueue& queue) {
  switch (msg.GetType()) {
  case EArchMsgType::UserInput: {
    ProcessUserInput(MakeMsg::GetParmUserInput(msg).x4_parm);
    break;
  case EArchMsgType::TimerTick: {
    Update(MakeMsg::GetParmTimerTick(msg).x4_parm, queue);
    break;
  }
  default:
    break;
  }
  }
  return EMessageReturn::Normal;
}

void CCredits::Draw() const { SCOPED_GRAPHICS_DEBUG_GROUP("CCredits::Draw", zeus::skGreen); }
void CCredits::Update(float, CArchitectureQueue& queue) {
  if (x14_ == 0) {
  } else if (x14_ == 1) {
  } else if (x14_ == 2) {
  } else if (x14_ == 3) {

  }
}

void CCredits::ProcessUserInput(const CFinalInput& input) {
  if (input.DA()) {
    x48_ = zeus::clamp(0.f, x48_ - ((x50_ * input.DeltaTime())), x4c_);
  } else {
    float leftY = input.ALeftY();
    float offset = 0.f;
    if (leftY < 0.f) {
      offset = -leftY;
      leftY = 0.f;
    }
    x48_ = zeus::clamp(0.f, x48_ - (leftY - offset) * 10.f * x50_ * input.DeltaTime(), x4c_);
  }
}

} // namespace urde::MP1
