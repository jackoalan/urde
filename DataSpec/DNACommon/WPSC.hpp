#pragma once

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
struct _WPSM {
  static constexpr ParticleType Type = ParticleType::WPSM;

  VectorElementFactory x0_IORN;
  VectorElementFactory x4_IVEC;
  VectorElementFactory x8_PSOV;
  ModVectorElementFactory xc_PSVM;
  IntElementFactory x14_PSLT;
  VectorElementFactory x18_PSCL;
  ColorElementFactory x1c_PCOL;
  VectorElementFactory x20_POFS;
  VectorElementFactory x24_OFST;

  RealElementFactory x30_TRAT;
  ChildResourceFactory<IDType> x34_APSM;
  ChildResourceFactory<IDType> x44_APS2;
  ChildResourceFactory<IDType> x54_ASW1;
  ChildResourceFactory<IDType> x64_ASW2;
  ChildResourceFactory<IDType> x74_ASW3;
  ChildResourceFactory<IDType> x84_OHEF;
  ChildResourceFactory<IDType> x94_COLR;
  uint32_t xa8_PJFX = ~0;
  RealElementFactory xac_RNGE;
  RealElementFactory xb0_FOFF;

  bool x10_VMD2 = false;
  bool x28_APSO = false;
  bool x29_HOMG = false;
  bool x2a_AP11 = false;
  bool x2b_AP21 = false;
  bool x2c_AS11 = false;
  bool x2d_AS12 = false;
  bool x2e_AS13 = false;
  bool xa4_EWTR = true;
  bool xa5_LWTR = true;
  bool xa6_SWTR = true;
  bool xunk_FC60 = false;
  bool xunk_SPS1 = false;
  bool xunk_SPS2 = false;

  template<typename _Func>
  void constexpr Enumerate(_Func f) {
    f(FOURCC('IORN'), x0_IORN);
    f(FOURCC('IVEC'), x4_IVEC);
    f(FOURCC('PSOV'), x8_PSOV);
    f(FOURCC('PSVM'), xc_PSVM);
    f(FOURCC('VMD2'), x10_VMD2);
    f(FOURCC('PSLT'), x14_PSLT);
    f(FOURCC('PSCL'), x18_PSCL);
    f(FOURCC('PCOL'), x1c_PCOL);
    f(FOURCC('POFS'), x20_POFS);
    f(FOURCC('OFST'), x24_OFST);
    f(FOURCC('APSO'), x28_APSO);
    f(FOURCC('HOMG'), x29_HOMG);
    f(FOURCC('AP11'), x2a_AP11);
    f(FOURCC('AP21'), x2b_AP21);
    f(FOURCC('AS11'), x2c_AS11);
    f(FOURCC('AS12'), x2d_AS12);
    f(FOURCC('AS13'), x2e_AS13);
    f(FOURCC('TRAT'), x30_TRAT);
    f(FOURCC('APSM'), x34_APSM);
    f(FOURCC('APS2'), x44_APS2);
    f(FOURCC('ASW1'), x54_ASW1);
    f(FOURCC('ASW2'), x64_ASW2);
    f(FOURCC('ASW3'), x74_ASW3);
    f(FOURCC('OHEF'), x84_OHEF);
    f(FOURCC('COLR'), x94_COLR);
    f(FOURCC('EWTR'), xa4_EWTR, true);
    f(FOURCC('LWTR'), xa5_LWTR, true);
    f(FOURCC('SWTR'), xa6_SWTR, true);
    f(FOURCC('PJFX'), xa8_PJFX);
    f(FOURCC('RNGE'), xac_RNGE);
    f(FOURCC('FOFF'), xb0_FOFF);
    f(FOURCC('FC60'), xunk_FC60);
    f(FOURCC('SPS1'), xunk_SPS1);
    f(FOURCC('SPS2'), xunk_SPS2);
  }

  template<typename _Func>
  bool constexpr Lookup(FourCC fcc, _Func f) {
    switch (fcc.toUint32()) {
    case SBIG('IORN'): f(x0_IORN); return true;
    case SBIG('IVEC'): f(x4_IVEC); return true;
    case SBIG('PSOV'): f(x8_PSOV); return true;
    case SBIG('PSVM'): f(xc_PSVM); return true;
    case SBIG('VMD2'): f(x10_VMD2); return true;
    case SBIG('PSLT'): f(x14_PSLT); return true;
    case SBIG('PSCL'): f(x18_PSCL); return true;
    case SBIG('PCOL'): f(x1c_PCOL); return true;
    case SBIG('POFS'): f(x20_POFS); return true;
    case SBIG('OFST'): f(x24_OFST); return true;
    case SBIG('APSO'): f(x28_APSO); return true;
    case SBIG('HOMG'): f(x29_HOMG); return true;
    case SBIG('AP11'): f(x2a_AP11); return true;
    case SBIG('AP21'): f(x2b_AP21); return true;
    case SBIG('AS11'): f(x2c_AS11); return true;
    case SBIG('AS12'): f(x2d_AS12); return true;
    case SBIG('AS13'): f(x2e_AS13); return true;
    case SBIG('TRAT'): f(x30_TRAT); return true;
    case SBIG('APSM'): f(x34_APSM); return true;
    case SBIG('APS2'): f(x44_APS2); return true;
    case SBIG('ASW1'): f(x54_ASW1); return true;
    case SBIG('ASW2'): f(x64_ASW2); return true;
    case SBIG('ASW3'): f(x74_ASW3); return true;
    case SBIG('OHEF'): f(x84_OHEF); return true;
    case SBIG('COLR'): f(x94_COLR); return true;
    case SBIG('EWTR'): f(xa4_EWTR); return true;
    case SBIG('LWTR'): f(xa5_LWTR); return true;
    case SBIG('SWTR'): f(xa6_SWTR); return true;
    case SBIG('PJFX'): f(xa8_PJFX); return true;
    case SBIG('RNGE'): f(xac_RNGE); return true;
    case SBIG('FOFF'): f(xb0_FOFF); return true;
    case SBIG('FC60'): f(xunk_FC60); return true;
    case SBIG('SPS1'): f(xunk_SPS1); return true;
    case SBIG('SPS2'): f(xunk_SPS2); return true;
    default: return false;
    }
  }
};
extern template struct PPImpl<_WPSM<UniqueID32>>;
extern template struct PPImpl<_WPSM<UniqueID64>>;
template <class IDType>
using WPSM = PPImpl<_WPSM<IDType>>;

template <class IDType>
bool ExtractWPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteWPSM(const WPSM<IDType>& wpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
