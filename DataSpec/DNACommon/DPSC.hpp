#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "DataSpec/DNACommon/DNACommon.hpp"
#include "DataSpec/DNACommon/ParticleCommon.hpp"

#include <athena/FileWriter.hpp>

namespace DataSpec {
class PAKEntryReadStream;
}

namespace hecl {
class ProjectPath;
}

namespace DataSpec::DNAParticle {

template <class IDType>
struct _DPSM {
  static constexpr ParticleType Type = ParticleType::DPSM;

  struct SQuadDescr {
    IntElementFactory x0_LFT;
    RealElementFactory x4_SZE;
    RealElementFactory x8_ROT;
    VectorElementFactory xc_OFF;
    ColorElementFactory x10_CLR;
    UVElementFactory<IDType> x14_TEX;
    bool x18_ADD = false;
  };

  SQuadDescr x0_quad;
  SQuadDescr x1c_quad;
  ChildResourceFactory<IDType> x38_DMDL;
  IntElementFactory x48_DLFT;
  VectorElementFactory x4c_DMOP;
  VectorElementFactory x50_DMRT;
  VectorElementFactory x54_DMSC;
  ColorElementFactory x58_DMCL;

  bool x5c_24_DMAB = false;
  bool x5c_25_DMOO = false;

  template<typename _Func>
  void constexpr Enumerate(_Func f) {
    f(FOURCC('1LFT'), x0_quad.x0_LFT);
    f(FOURCC('1SZE'), x0_quad.x4_SZE);
    f(FOURCC('1ROT'), x0_quad.x8_ROT);
    f(FOURCC('1OFF'), x0_quad.xc_OFF);
    f(FOURCC('1CLR'), x0_quad.x10_CLR);
    f(FOURCC('1TEX'), x0_quad.x14_TEX);
    f(FOURCC('1ADD'), x0_quad.x18_ADD);
    f(FOURCC('2LFT'), x1c_quad.x0_LFT);
    f(FOURCC('2SZE'), x1c_quad.x4_SZE);
    f(FOURCC('2ROT'), x1c_quad.x8_ROT);
    f(FOURCC('2OFF'), x1c_quad.xc_OFF);
    f(FOURCC('2CLR'), x1c_quad.x10_CLR);
    f(FOURCC('2TEX'), x1c_quad.x14_TEX);
    f(FOURCC('2ADD'), x1c_quad.x18_ADD);
    f(FOURCC('DMDL'), x38_DMDL);
    f(FOURCC('DLFT'), x48_DLFT);
    f(FOURCC('DMOP'), x4c_DMOP);
    f(FOURCC('DMRT'), x50_DMRT);
    f(FOURCC('DMSC'), x54_DMSC);
    f(FOURCC('DMCL'), x58_DMCL);
    f(FOURCC('DMAB'), x5c_24_DMAB);
    f(FOURCC('DMOO'), x5c_25_DMOO);
  }

  template<typename _Func>
  bool constexpr Lookup(FourCC fcc, _Func f) {
    switch (fcc.toUint32()) {
    case SBIG('1LFT'): f(x0_quad.x0_LFT); return true;
    case SBIG('1SZE'): f(x0_quad.x4_SZE); return true;
    case SBIG('1ROT'): f(x0_quad.x8_ROT); return true;
    case SBIG('1OFF'): f(x0_quad.xc_OFF); return true;
    case SBIG('1CLR'): f(x0_quad.x10_CLR); return true;
    case SBIG('1TEX'): f(x0_quad.x14_TEX); return true;
    case SBIG('1ADD'): f(x0_quad.x18_ADD); return true;
    case SBIG('2LFT'): f(x1c_quad.x0_LFT); return true;
    case SBIG('2SZE'): f(x1c_quad.x4_SZE); return true;
    case SBIG('2ROT'): f(x1c_quad.x8_ROT); return true;
    case SBIG('2OFF'): f(x1c_quad.xc_OFF); return true;
    case SBIG('2CLR'): f(x1c_quad.x10_CLR); return true;
    case SBIG('2TEX'): f(x1c_quad.x14_TEX); return true;
    case SBIG('2ADD'): f(x1c_quad.x18_ADD); return true;
    case SBIG('DMDL'): f(x38_DMDL); return true;
    case SBIG('DLFT'): f(x48_DLFT); return true;
    case SBIG('DMOP'): f(x4c_DMOP); return true;
    case SBIG('DMRT'): f(x50_DMRT); return true;
    case SBIG('DMSC'): f(x54_DMSC); return true;
    case SBIG('DMCL'): f(x58_DMCL); return true;
    default: return false;
    }
  }
};
extern template struct PPImpl<_DPSM<UniqueID32>>;
extern template struct PPImpl<_DPSM<UniqueID64>>;
template <class IDType>
using DPSM = PPImpl<_DPSM<IDType>>;

template <class IDType>
bool ExtractDPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteDPSM(const DPSM<IDType>& dpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
