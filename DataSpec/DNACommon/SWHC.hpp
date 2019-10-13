#pragma once

#include <vector>

#include "DataSpec/DNACommon/DNACommon.hpp"
#include "DataSpec/DNACommon/ParticleCommon.hpp"

namespace DataSpec {
class PAKEntryReadStream;
}

namespace hecl {
class ProjectPath;
}

namespace DataSpec::DNAParticle {

template <class IDType>
struct _SWSH {
  enum EPP : uint32_t {
    PSLT = SBIG('PSLT'), // x0
    TIME = SBIG('TIME'), // x4
    LRAD = SBIG('LRAD'), // x8
    RRAD = SBIG('RRAD'), // xc
    LENG = SBIG('LENG'), // x10
    COLR = SBIG('COLR'), // x14
    SIDE = SBIG('SIDE'), // x18
    IROT = SBIG('IROT'), // x1c
    ROTM = SBIG('ROTM'), // x20
    POFS = SBIG('POFS'), // x24
    IVEL = SBIG('IVEL'), // x28
    NPOS = SBIG('NPOS'), // x2c
    VELM = SBIG('VELM'), // x30
    VLM2 = SBIG('VLM2'), // x34
    SPLN = SBIG('SPLN'), // x38
    TEXR = SBIG('TEXR'), // x3c
    TSPN = SBIG('TSPN'), // x40
    LLRD = SBIG('LLRD'), // x44_24
    CROS = SBIG('CROS'), // x44_25
    VLS1 = SBIG('VLS1'), // x44_26
    VLS2 = SBIG('VLS2'), // x44_27
    SROT = SBIG('SROT'), // x44_28
    WIRE = SBIG('WIRE'), // x44_29
    TEXW = SBIG('TEXW'), // x44_30
    AALP = SBIG('AALP'), // x44_31
    ZBUF = SBIG('ZBUF'), // x45_24
    ORNT = SBIG('ORNT'), // x45_25
    CRND = SBIG('CRND'), // x45_26
  };
  using Properties = PPSet<
      ParticleType::SWSH,
      std::tuple<
          PP<PSLT, IntElementFactory>,
          PP<TIME, RealElementFactory>,
          PP<LRAD, RealElementFactory>,
          PP<RRAD, RealElementFactory>,
          PP<LENG, IntElementFactory>,
          PP<COLR, ColorElementFactory>,
          PP<SIDE, IntElementFactory>,
          PP<IROT, RealElementFactory>,
          PP<ROTM, RealElementFactory>,
          PP<POFS, VectorElementFactory>,
          PP<IVEL, VectorElementFactory>,
          PP<NPOS, VectorElementFactory>,
          PP<VELM, ModVectorElementFactory>,
          PP<VLM2, ModVectorElementFactory>,
          PP<SPLN, IntElementFactory>,
          PP<TEXR, UVElementFactory<IDType>>,
          PP<TSPN, IntElementFactory>,
          PP<LLRD, bool>,
          PP<CROS, bool, true>,
          PP<VLS1, bool>,
          PP<VLS2, bool>,
          PP<SROT, bool>,
          PP<WIRE, bool>,
          PP<TEXW, bool>,
          PP<AALP, bool>,
          PP<ZBUF, bool>,
          PP<ORNT, bool>,
          PP<CRND, bool>>,
      AALP, COLR, CRND, CROS, IROT, IVEL, LENG, LLRD, LRAD, NPOS, ORNT, POFS, PSLT, ROTM, RRAD, SIDE, SPLN, SROT,
      TEXR, TEXW, TIME, TSPN, VELM, VLM2, VLS1, VLS2, WIRE, ZBUF>;
};

template <class IDType>
using SWSH = PPImpl<_SWSH<IDType>>;

template <class IDType>
bool ExtractSWSH(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteSWSH(const SWSH<IDType>& gpsm, const hecl::ProjectPath& outPath);
} // namespace DataSpec::DNAParticle
