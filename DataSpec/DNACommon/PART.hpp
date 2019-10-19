#pragma once

#include <cstdint>
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
struct _GPSM {
  static constexpr ParticleType Type = ParticleType::GPSM;

  VectorElementFactory x0_PSIV;
  ModVectorElementFactory x4_PSVM;
  VectorElementFactory x8_PSOV;
  IntElementFactory xc_PSLT;
  IntElementFactory x10_PSWT;
  RealElementFactory x14_PSTS;
  VectorElementFactory x18_POFS;
  IntElementFactory x1c_SEED;
  RealElementFactory x20_LENG;
  RealElementFactory x24_WIDT;
  IntElementFactory x28_MAXP;
  RealElementFactory x2c_GRTE;
  ColorElementFactory x30_COLR;
  IntElementFactory x34_LTME;
  VectorElementFactory x38_ILOC;
  VectorElementFactory x3c_IVEC;
  EmitterElementFactory x40_EMTR;
  IntElementFactory x48_MBSP;
  RealElementFactory x4c_SIZE;
  RealElementFactory x50_ROTA;
  UVElementFactory<IDType> x54_TEXR;
  UVElementFactory<IDType> x58_TIND;
  ChildResourceFactory<IDType> x5c_PMDL;
  VectorElementFactory x6c_PMOP;
  VectorElementFactory x70_PMRT;
  VectorElementFactory x74_PMSC;
  ColorElementFactory x78_PMCL;
  ModVectorElementFactory x7c_VEL1;
  ModVectorElementFactory x80_VEL2;
  ModVectorElementFactory x84_VEL3;
  ModVectorElementFactory x88_VEL4;
  ChildResourceFactory<IDType> x8c_ICTS;
  IntElementFactory x9c_NCSY;
  IntElementFactory xa0_CSSD;
  ChildResourceFactory<IDType> xa4_IDTS;
  IntElementFactory xb4_NDSY;
  ChildResourceFactory<IDType> xb8_IITS;
  IntElementFactory xc8_PISY;
  IntElementFactory xcc_SISY;
  SpawnSystemKeyframeData<IDType> xd0_KSSM;
  ChildResourceFactory<IDType> xd4_SSWH;
  IntElementFactory xe4_SSSD;
  VectorElementFactory xe8_SSPO;
  IntElementFactory xf8_SESD;
  VectorElementFactory xfc_SEPO;
  ChildResourceFactory<IDType> xec_PMLC;
  IntElementFactory x100_LTYP;
  ColorElementFactory x104_LCLR;
  RealElementFactory x108_LINT;
  VectorElementFactory x10c_LOFF;
  VectorElementFactory x110_LDIR;
  IntElementFactory x114_LFOT;
  RealElementFactory x118_LFOR;
  RealElementFactory x11c_LSLA;

  /* 0-00 additions */
  ChildResourceFactory<IDType> xd8_SELC;
  RealElementFactory x10c_ADV1;
  RealElementFactory x110_ADV2;
  RealElementFactory x114_ADV3;
  RealElementFactory x118_ADV4;
  RealElementFactory x11c_ADV5;
  RealElementFactory x120_ADV6;
  RealElementFactory x124_ADV7;
  RealElementFactory x128_ADV8;

  bool x44_28_SORT = false;
  bool x44_30_MBLR = false;
  bool x44_24_LINE = false;
  bool x44_29_LIT_ = false;
  bool x44_26_AAPH = false;
  bool x44_27_ZBUF = false;
  bool x44_25_FXLL = false;
  bool x44_31_PMAB = false;
  bool x45_29_VMD4 = false;
  bool x45_28_VMD3 = false;
  bool x45_27_VMD2 = false;
  bool x45_26_VMD1 = false;
  bool x45_31_OPTS = false;
  bool x45_24_PMUS = false;
  bool x45_25_PMOO = true;
  bool x45_30_CIND = false;
  
  bool x30_30_ORNT = false;
  bool x30_31_RSOP = false;

  template<typename _Func>
  void constexpr Enumerate(_Func f) {
    f(FOURCC('PSIV'), x0_PSIV);
    f(FOURCC('PSVM'), x4_PSVM);
    f(FOURCC('PSOV'), x8_PSOV);
    f(FOURCC('PSLT'), xc_PSLT);
    f(FOURCC('PSWT'), x10_PSWT);
    f(FOURCC('PSTS'), x14_PSTS);
    f(FOURCC('POFS'), x18_POFS);
    f(FOURCC('SEED'), x1c_SEED);
    f(FOURCC('LENG'), x20_LENG);
    f(FOURCC('WIDT'), x24_WIDT);
    f(FOURCC('MAXP'), x28_MAXP);
    f(FOURCC('GRTE'), x2c_GRTE);
    f(FOURCC('COLR'), x30_COLR);
    f(FOURCC('LTME'), x34_LTME);
    f(FOURCC('ILOC'), x38_ILOC);
    f(FOURCC('IVEC'), x3c_IVEC);
    f(FOURCC('EMTR'), x40_EMTR);
    f(FOURCC('MBSP'), x48_MBSP);
    f(FOURCC('SIZE'), x4c_SIZE);
    f(FOURCC('ROTA'), x50_ROTA);
    f(FOURCC('TEXR'), x54_TEXR);
    f(FOURCC('TIND'), x58_TIND);
    f(FOURCC('PMDL'), x5c_PMDL);
    f(FOURCC('PMOP'), x6c_PMOP);
    f(FOURCC('PMRT'), x70_PMRT);
    f(FOURCC('PMSC'), x74_PMSC);
    f(FOURCC('PMCL'), x78_PMCL);
    f(FOURCC('VEL1'), x7c_VEL1);
    f(FOURCC('VEL2'), x80_VEL2);
    f(FOURCC('VEL3'), x84_VEL3);
    f(FOURCC('VEL4'), x88_VEL4);
    f(FOURCC('ICTS'), x8c_ICTS);
    f(FOURCC('NCSY'), x9c_NCSY);
    f(FOURCC('CSSD'), xa0_CSSD);
    f(FOURCC('IDTS'), xa4_IDTS);
    f(FOURCC('NDSY'), xb4_NDSY);
    f(FOURCC('IITS'), xb8_IITS);
    f(FOURCC('PISY'), xc8_PISY);
    f(FOURCC('SISY'), xcc_SISY);
    f(FOURCC('KSSM'), xd0_KSSM);
    f(FOURCC('SSWH'), xd4_SSWH);
    f(FOURCC('SSSD'), xe4_SSSD);
    f(FOURCC('SSPO'), xe8_SSPO);
    f(FOURCC('SESD'), xf8_SESD);
    f(FOURCC('SEPO'), xfc_SEPO);
    f(FOURCC('PMLC'), xec_PMLC);
    f(FOURCC('LTYP'), x100_LTYP);
    f(FOURCC('LCLR'), x104_LCLR);
    f(FOURCC('LINT'), x108_LINT);
    f(FOURCC('LOFF'), x10c_LOFF);
    f(FOURCC('LDIR'), x110_LDIR);
    f(FOURCC('LFOT'), x114_LFOT);
    f(FOURCC('LFOR'), x118_LFOR);
    f(FOURCC('LSLA'), x11c_LSLA);
    f(FOURCC('SELC'), xd8_SELC);
    f(FOURCC('ADV1'), x10c_ADV1);
    f(FOURCC('ADV2'), x110_ADV2);
    f(FOURCC('ADV3'), x114_ADV3);
    f(FOURCC('ADV4'), x118_ADV4);
    f(FOURCC('ADV5'), x11c_ADV5);
    f(FOURCC('ADV6'), x120_ADV6);
    f(FOURCC('ADV7'), x124_ADV7);
    f(FOURCC('ADV8'), x128_ADV8);
    f(FOURCC('SORT'), x44_28_SORT);
    f(FOURCC('MBLR'), x44_30_MBLR);
    f(FOURCC('LINE'), x44_24_LINE);
    f(FOURCC('LIT_'), x44_29_LIT_);
    f(FOURCC('AAPH'), x44_26_AAPH);
    f(FOURCC('ZBUF'), x44_27_ZBUF);
    f(FOURCC('FXLL'), x44_25_FXLL);
    f(FOURCC('PMAB'), x44_31_PMAB);
    f(FOURCC('VMD4'), x45_29_VMD4);
    f(FOURCC('VMD3'), x45_28_VMD3);
    f(FOURCC('VMD2'), x45_27_VMD2);
    f(FOURCC('VMD1'), x45_26_VMD1);
    f(FOURCC('OPTS'), x45_31_OPTS);
    f(FOURCC('PMUS'), x45_24_PMUS);
    f(FOURCC('PMOO'), x45_25_PMOO, true);
    f(FOURCC('CIND'), x45_30_CIND);
    f(FOURCC('ORNT'), x30_30_ORNT);
    f(FOURCC('RSOP'), x30_31_RSOP);
  }

  template<typename _Func>
  bool constexpr Lookup(FourCC fcc, _Func f) {
    switch (fcc.toUint32()) {
    case SBIG('PSIV'): f(x0_PSIV); return true;
    case SBIG('PSVM'): f(x4_PSVM); return true;
    case SBIG('PSOV'): f(x8_PSOV); return true;
    case SBIG('PSLT'): f(xc_PSLT); return true;
    case SBIG('PSWT'): f(x10_PSWT); return true;
    case SBIG('PSTS'): f(x14_PSTS); return true;
    case SBIG('POFS'): f(x18_POFS); return true;
    case SBIG('SEED'): f(x1c_SEED); return true;
    case SBIG('LENG'): f(x20_LENG); return true;
    case SBIG('WIDT'): f(x24_WIDT); return true;
    case SBIG('MAXP'): f(x28_MAXP); return true;
    case SBIG('GRTE'): f(x2c_GRTE); return true;
    case SBIG('COLR'): f(x30_COLR); return true;
    case SBIG('LTME'): f(x34_LTME); return true;
    case SBIG('ILOC'): f(x38_ILOC); return true;
    case SBIG('IVEC'): f(x3c_IVEC); return true;
    case SBIG('EMTR'): f(x40_EMTR); return true;
    case SBIG('MBSP'): f(x48_MBSP); return true;
    case SBIG('SIZE'): f(x4c_SIZE); return true;
    case SBIG('ROTA'): f(x50_ROTA); return true;
    case SBIG('TEXR'): f(x54_TEXR); return true;
    case SBIG('TIND'): f(x58_TIND); return true;
    case SBIG('PMDL'): f(x5c_PMDL); return true;
    case SBIG('PMOP'): f(x6c_PMOP); return true;
    case SBIG('PMRT'): f(x70_PMRT); return true;
    case SBIG('PMSC'): f(x74_PMSC); return true;
    case SBIG('PMCL'): f(x78_PMCL); return true;
    case SBIG('VEL1'): f(x7c_VEL1); return true;
    case SBIG('VEL2'): f(x80_VEL2); return true;
    case SBIG('VEL3'): f(x84_VEL3); return true;
    case SBIG('VEL4'): f(x88_VEL4); return true;
    case SBIG('ICTS'): f(x8c_ICTS); return true;
    case SBIG('NCSY'): f(x9c_NCSY); return true;
    case SBIG('CSSD'): f(xa0_CSSD); return true;
    case SBIG('IDTS'): f(xa4_IDTS); return true;
    case SBIG('NDSY'): f(xb4_NDSY); return true;
    case SBIG('IITS'): f(xb8_IITS); return true;
    case SBIG('PISY'): f(xc8_PISY); return true;
    case SBIG('SISY'): f(xcc_SISY); return true;
    case SBIG('KSSM'): f(xd0_KSSM); return true;
    case SBIG('SSWH'): f(xd4_SSWH); return true;
    case SBIG('SSSD'): f(xe4_SSSD); return true;
    case SBIG('SSPO'): f(xe8_SSPO); return true;
    case SBIG('SESD'): f(xf8_SESD); return true;
    case SBIG('SEPO'): f(xfc_SEPO); return true;
    case SBIG('PMLC'): f(xec_PMLC); return true;
    case SBIG('LTYP'): f(x100_LTYP); return true;
    case SBIG('LCLR'): f(x104_LCLR); return true;
    case SBIG('LINT'): f(x108_LINT); return true;
    case SBIG('LOFF'): f(x10c_LOFF); return true;
    case SBIG('LDIR'): f(x110_LDIR); return true;
    case SBIG('LFOT'): f(x114_LFOT); return true;
    case SBIG('LFOR'): f(x118_LFOR); return true;
    case SBIG('LSLA'): f(x11c_LSLA); return true;
    case SBIG('SELC'): f(xd8_SELC); return true;
    case SBIG('ADV1'): f(x10c_ADV1); return true;
    case SBIG('ADV2'): f(x110_ADV2); return true;
    case SBIG('ADV3'): f(x114_ADV3); return true;
    case SBIG('ADV4'): f(x118_ADV4); return true;
    case SBIG('ADV5'): f(x11c_ADV5); return true;
    case SBIG('ADV6'): f(x120_ADV6); return true;
    case SBIG('ADV7'): f(x124_ADV7); return true;
    case SBIG('ADV8'): f(x128_ADV8); return true;
    case SBIG('SORT'): f(x44_28_SORT); return true;
    case SBIG('MBLR'): f(x44_30_MBLR); return true;
    case SBIG('LINE'): f(x44_24_LINE); return true;
    case SBIG('LIT_'): f(x44_29_LIT_); return true;
    case SBIG('AAPH'): f(x44_26_AAPH); return true;
    case SBIG('ZBUF'): f(x44_27_ZBUF); return true;
    case SBIG('FXLL'): f(x44_25_FXLL); return true;
    case SBIG('PMAB'): f(x44_31_PMAB); return true;
    case SBIG('VMD4'): f(x45_29_VMD4); return true;
    case SBIG('VMD3'): f(x45_28_VMD3); return true;
    case SBIG('VMD2'): f(x45_27_VMD2); return true;
    case SBIG('VMD1'): f(x45_26_VMD1); return true;
    case SBIG('OPTS'): f(x45_31_OPTS); return true;
    case SBIG('PMUS'): f(x45_24_PMUS); return true;
    case SBIG('PMOO'): f(x45_25_PMOO); return true;
    case SBIG('CIND'): f(x45_30_CIND); return true;
    case SBIG('ORNT'): f(x30_30_ORNT); return true;
    case SBIG('RSOP'): f(x30_31_RSOP); return true;
    default: return false;
    }
  }
};
template <class IDType>
using GPSM = PPImpl<_GPSM<IDType>>;

template <class IDType>
bool ExtractGPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteGPSM(const GPSM<IDType>& gpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
