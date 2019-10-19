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
  static constexpr ParticleType Type = ParticleType::ELSM;

  IntElementFactory x0_LIFE;
  IntElementFactory x4_SLIF;
  RealElementFactory x8_GRAT;
  IntElementFactory xc_SCNT;
  IntElementFactory x10_SSEG;
  ColorElementFactory x14_COLR;
  EmitterElementFactory x18_IEMT;
  EmitterElementFactory x1c_FEMT;
  RealElementFactory x20_AMPL;
  RealElementFactory x24_AMPD;
  RealElementFactory x28_LWD1;
  RealElementFactory x2c_LWD2;
  RealElementFactory x30_LWD3;
  ColorElementFactory x34_LCL1;
  ColorElementFactory x38_LCL2;
  ColorElementFactory x3c_LCL3;
  ChildResourceFactory<IDType> x40_SSWH;
  ChildResourceFactory<IDType> x50_GPSM;
  ChildResourceFactory<IDType> x60_EPSM;
  bool x70_ZERY = false;

  template<typename _Func>
  void constexpr Enumerate(_Func f) {
    f(FOURCC('LIFE'), x0_LIFE);
    f(FOURCC('SLIF'), x4_SLIF);
    f(FOURCC('GRAT'), x8_GRAT);
    f(FOURCC('SCNT'), xc_SCNT);
    f(FOURCC('SSEG'), x10_SSEG);
    f(FOURCC('COLR'), x14_COLR);
    f(FOURCC('IEMT'), x18_IEMT);
    f(FOURCC('FEMT'), x1c_FEMT);
    f(FOURCC('AMPL'), x20_AMPL);
    f(FOURCC('AMPD'), x24_AMPD);
    f(FOURCC('LWD1'), x28_LWD1);
    f(FOURCC('LWD2'), x2c_LWD2);
    f(FOURCC('LWD3'), x30_LWD3);
    f(FOURCC('LCL1'), x34_LCL1);
    f(FOURCC('LCL2'), x38_LCL2);
    f(FOURCC('LCL3'), x3c_LCL3);
    f(FOURCC('SSWH'), x40_SSWH);
    f(FOURCC('GPSM'), x50_GPSM);
    f(FOURCC('EPSM'), x60_EPSM);
    f(FOURCC('ZERY'), x70_ZERY);
  }

  template<typename _Func>
  bool constexpr Lookup(FourCC fcc, _Func f) {
    switch (fcc.toUint32()) {
    case SBIG('LIFE'): f(x0_LIFE); return true;
    case SBIG('SLIF'): f(x4_SLIF); return true;
    case SBIG('GRAT'): f(x8_GRAT); return true;
    case SBIG('SCNT'): f(xc_SCNT); return true;
    case SBIG('SSEG'): f(x10_SSEG); return true;
    case SBIG('COLR'): f(x14_COLR); return true;
    case SBIG('IEMT'): f(x18_IEMT); return true;
    case SBIG('FEMT'): f(x1c_FEMT); return true;
    case SBIG('AMPL'): f(x20_AMPL); return true;
    case SBIG('AMPD'): f(x24_AMPD); return true;
    case SBIG('LWD1'): f(x28_LWD1); return true;
    case SBIG('LWD2'): f(x2c_LWD2); return true;
    case SBIG('LWD3'): f(x30_LWD3); return true;
    case SBIG('LCL1'): f(x34_LCL1); return true;
    case SBIG('LCL2'): f(x38_LCL2); return true;
    case SBIG('LCL3'): f(x3c_LCL3); return true;
    case SBIG('SSWH'): f(x40_SSWH); return true;
    case SBIG('GPSM'): f(x50_GPSM); return true;
    case SBIG('EPSM'): f(x60_EPSM); return true;
    case SBIG('ZERY'): f(x70_ZERY); return true;
    default: return false;
    }
  }
};
template <class IDType>
using ELSM = PPImpl<_ELSM<IDType>>;

template <class IDType>
bool ExtractELSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteELSM(const ELSM<IDType>& elsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
