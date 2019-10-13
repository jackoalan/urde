#pragma once

#include <vector>

#include "DataSpec/DNACommon/ParticleCommon.hpp"
#include "DataSpec/DNACommon/PAK.hpp"

#include <athena/FileWriter.hpp>

namespace hecl {
class ProjectPath;
}

namespace DataSpec::DNAParticle {

template <class IDType>
struct _ELSM {
  enum EPP : uint32_t {
    LIFE = SBIG('LIFE'), // x0
    SLIF = SBIG('SLIF'), // x4
    GRAT = SBIG('GRAT'), // x8
    SCNT = SBIG('SCNT'), // xc
    SSEG = SBIG('SSEG'), // x10
    COLR = SBIG('COLR'), // x14
    IEMT = SBIG('IEMT'), // x18
    FEMT = SBIG('FEMT'), // x1c
    AMPL = SBIG('AMPL'), // x20
    AMPD = SBIG('AMPD'), // x24
    LWD1 = SBIG('LWD1'), // x28
    LWD2 = SBIG('LWD2'), // x2c
    LWD3 = SBIG('LWD3'), // x30
    LCL1 = SBIG('LCL1'), // x34
    LCL2 = SBIG('LCL2'), // x38
    LCL3 = SBIG('LCL3'), // x3c
    SSWH = SBIG('SSWH'), // x40
    GPSM = SBIG('GPSM'), // x50
    EPSM = SBIG('EPSM'), // x60
    ZERY = SBIG('ZERY'), // x70
  };
  using Properties = PPSet<
      ParticleType::ELSM,
      std::tuple<
          PP<LIFE, IntElementFactory>,
          PP<SLIF, IntElementFactory>,
          PP<GRAT, RealElementFactory>,
          PP<SCNT, IntElementFactory>,
          PP<SSEG, IntElementFactory>,
          PP<COLR, ColorElementFactory>,
          PP<IEMT, EmitterElementFactory>,
          PP<FEMT, EmitterElementFactory>,
          PP<AMPL, RealElementFactory>,
          PP<AMPD, RealElementFactory>,
          PP<LWD1, RealElementFactory>,
          PP<LWD2, RealElementFactory>,
          PP<LWD3, RealElementFactory>,
          PP<LCL1, ColorElementFactory>,
          PP<LCL2, ColorElementFactory>,
          PP<LCL3, ColorElementFactory>,
          PP<SSWH, ChildResourceFactory<IDType>>,
          PP<GPSM, ChildResourceFactory<IDType>>,
          PP<EPSM, ChildResourceFactory<IDType>>,
          PP<ZERY, bool>>,
      AMPD, AMPL, COLR, EPSM, FEMT, GPSM, GRAT, IEMT, LCL1, LCL2, LCL3, LIFE, LWD1, LWD2, LWD3, SCNT, SLIF, SSEG,
      SSWH, ZERY>;
};

template <class IDType>
using ELSM = PPImpl<_ELSM<IDType>>;

template <class IDType>
bool ExtractELSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteELSM(const ELSM<IDType>& elsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
