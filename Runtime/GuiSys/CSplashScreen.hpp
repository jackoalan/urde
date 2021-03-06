#pragma once

#include "Runtime/CIOWin.hpp"
#include "Runtime/CToken.hpp"
#include "Runtime/Graphics/CTexture.hpp"
#include "Runtime/Graphics/Shaders/CTexturedQuadFilter.hpp"

namespace urde {

class CSplashScreen : public CIOWin {
public:
  enum class ESplashScreen { Nintendo, Retro, Dolby };
  enum class EProgressivePhase { Before, During, After };

private:
  ESplashScreen x14_which;
  float x18_splashTimeout = 2.f;
  // float x1c_progSelectionTimeout = 0.f;
  // EProgressivePhase x20_progressivePhase = EProgressivePhase::Before;
  // bool x24_progressiveSelection = true;
  bool x25_textureLoaded = false;
  CTexturedQuadFilterAlpha m_quad;

public:
  explicit CSplashScreen(ESplashScreen);
  EMessageReturn OnMessage(const CArchitectureMessage&, CArchitectureQueue&) override;
  void Draw() const override;
};

} // namespace urde
