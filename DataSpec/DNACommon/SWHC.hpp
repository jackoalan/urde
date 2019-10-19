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
  static constexpr ParticleType Type = ParticleType::SWSH;

  IntElementFactory x0_PSLT;
  RealElementFactory x4_TIME;
  RealElementFactory x8_LRAD;
  RealElementFactory xc_RRAD;
  IntElementFactory x10_LENG;
  ColorElementFactory x14_COLR;
  IntElementFactory x18_SIDE;
  RealElementFactory x1c_IROT;
  RealElementFactory x20_ROTM;
  VectorElementFactory x24_POFS;
  VectorElementFactory x28_IVEL;
  VectorElementFactory x2c_NPOS;
  ModVectorElementFactory x30_VELM;
  ModVectorElementFactory x34_VLM2;
  IntElementFactory x38_SPLN;
  UVElementFactory<IDType> x3c_TEXR;
  IntElementFactory x40_TSPN;
  bool x44_24_LLRD = false;
  bool x44_25_CROS = true;
  bool x44_26_VLS1 = false;
  bool x44_27_VLS2 = false;
  bool x44_28_SROT = false;
  bool x44_29_WIRE = false;
  bool x44_30_TEXW = false;
  bool x44_31_AALP = false;
  bool x45_24_ZBUF = false;
  bool x45_25_ORNT = false;
  bool x45_26_CRND = false;

  template<typename _Func>
  void constexpr Enumerate(_Func f) {
    f(FOURCC('PSLT'), x0_PSLT);
    f(FOURCC('TIME'), x4_TIME);
    f(FOURCC('LRAD'), x8_LRAD);
    f(FOURCC('RRAD'), xc_RRAD);
    f(FOURCC('LENG'), x10_LENG);
    f(FOURCC('COLR'), x14_COLR);
    f(FOURCC('SIDE'), x18_SIDE);
    f(FOURCC('IROT'), x1c_IROT);
    f(FOURCC('ROTM'), x20_ROTM);
    f(FOURCC('POFS'), x24_POFS);
    f(FOURCC('IVEL'), x28_IVEL);
    f(FOURCC('NPOS'), x2c_NPOS);
    f(FOURCC('VELM'), x30_VELM);
    f(FOURCC('VLM2'), x34_VLM2);
    f(FOURCC('SPLN'), x38_SPLN);
    f(FOURCC('TEXR'), x3c_TEXR);
    f(FOURCC('TSPN'), x40_TSPN);
    f(FOURCC('LLRD'), x44_24_LLRD);
    f(FOURCC('CROS'), x44_25_CROS, true);
    f(FOURCC('VLS1'), x44_26_VLS1);
    f(FOURCC('VLS2'), x44_27_VLS2);
    f(FOURCC('SROT'), x44_28_SROT);
    f(FOURCC('WIRE'), x44_29_WIRE);
    f(FOURCC('TEXW'), x44_30_TEXW);
    f(FOURCC('AALP'), x44_31_AALP);
    f(FOURCC('ZBUF'), x45_24_ZBUF);
    f(FOURCC('ORNT'), x45_25_ORNT);
    f(FOURCC('CRND'), x45_26_CRND);
  }

  template<typename _Func>
  bool constexpr Lookup(FourCC fcc, _Func f) {
    switch (fcc.toUint32()) {
    case SBIG('PSLT'): f(x0_PSLT); return true;
    case SBIG('TIME'): f(x4_TIME); return true;
    case SBIG('LRAD'): f(x8_LRAD); return true;
    case SBIG('RRAD'): f(xc_RRAD); return true;
    case SBIG('LENG'): f(x10_LENG); return true;
    case SBIG('COLR'): f(x14_COLR); return true;
    case SBIG('SIDE'): f(x18_SIDE); return true;
    case SBIG('IROT'): f(x1c_IROT); return true;
    case SBIG('ROTM'): f(x20_ROTM); return true;
    case SBIG('POFS'): f(x24_POFS); return true;
    case SBIG('IVEL'): f(x28_IVEL); return true;
    case SBIG('NPOS'): f(x2c_NPOS); return true;
    case SBIG('VELM'): f(x30_VELM); return true;
    case SBIG('VLM2'): f(x34_VLM2); return true;
    case SBIG('SPLN'): f(x38_SPLN); return true;
    case SBIG('TEXR'): f(x3c_TEXR); return true;
    case SBIG('TSPN'): f(x40_TSPN); return true;
    case SBIG('LLRD'): f(x44_24_LLRD); return true;
    case SBIG('CROS'): f(x44_25_CROS); return true;
    case SBIG('VLS1'): f(x44_26_VLS1); return true;
    case SBIG('VLS2'): f(x44_27_VLS2); return true;
    case SBIG('SROT'): f(x44_28_SROT); return true;
    case SBIG('WIRE'): f(x44_29_WIRE); return true;
    case SBIG('TEXW'): f(x44_30_TEXW); return true;
    case SBIG('AALP'): f(x44_31_AALP); return true;
    case SBIG('ZBUF'): f(x45_24_ZBUF); return true;
    case SBIG('ORNT'): f(x45_25_ORNT); return true;
    case SBIG('CRND'): f(x45_26_CRND); return true;
    default: return false;
    }
  }
};
template <class IDType>
using SWSH = PPImpl<_SWSH<IDType>>;

template <class IDType>
bool ExtractSWSH(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteSWSH(const SWSH<IDType>& gpsm, const hecl::ProjectPath& outPath);
} // namespace DataSpec::DNAParticle
