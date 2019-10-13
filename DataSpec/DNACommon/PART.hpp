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
  enum EPP : uint32_t {
    PSIV = SBIG('PSIV'), // x0
    PSVM = SBIG('PSVM'), // x4
    PSOV = SBIG('PSOV'), // x8
    PSLT = SBIG('PSLT'), // xc
    PSWT = SBIG('PSWT'), // x10
    PSTS = SBIG('PSTS'), // x14
    POFS = SBIG('POFS'), // x18
    SEED = SBIG('SEED'), // x1c
    LENG = SBIG('LENG'), // x20
    WIDT = SBIG('WIDT'), // x24
    MAXP = SBIG('MAXP'), // x28
    GRTE = SBIG('GRTE'), // x2c
    COLR = SBIG('COLR'), // x30
    LTME = SBIG('LTME'), // x34
    ILOC = SBIG('ILOC'), // x38
    IVEC = SBIG('IVEC'), // x3c
    EMTR = SBIG('EMTR'), // x40
    SORT = SBIG('SORT'), // x44_28
    MBLR = SBIG('MBLR'), // x44_30
    LINE = SBIG('LINE'), // x44_24
    LIT_ = SBIG('LIT_'), // x44_29
    AAPH = SBIG('AAPH'), // x44_26
    ZBUF = SBIG('ZBUF'), // x44_27
    FXLL = SBIG('FXLL'), // x44_25
    PMAB = SBIG('PMAB'), // x44_31
    VMD4 = SBIG('VMD4'), // x45_29
    VMD3 = SBIG('VMD3'), // x45_28
    VMD2 = SBIG('VMD2'), // x45_27
    VMD1 = SBIG('VMD1'), // x45_26
    OPTS = SBIG('OPTS'), // x45_31
    PMUS = SBIG('PMUS'), // x45_24
    PMOO = SBIG('PMOO'), // x45_25
    CIND = SBIG('CIND'), // x45_30
    MBSP = SBIG('MBSP'), // x48
    SIZE = SBIG('SIZE'), // x4c
    ROTA = SBIG('ROTA'), // x50
    TEXR = SBIG('TEXR'), // x54
    TIND = SBIG('TIND'), // x58
    PMDL = SBIG('PMDL'), // x5c
    PMOP = SBIG('PMOP'), // x6c
    PMRT = SBIG('PMRT'), // x70
    PMSC = SBIG('PMSC'), // x74
    PMCL = SBIG('PMCL'), // x78
    VEL1 = SBIG('VEL1'), // x7c
    VEL2 = SBIG('VEL2'), // x80
    VEL3 = SBIG('VEL3'), // x84
    VEL4 = SBIG('VEL4'), // x88
    ICTS = SBIG('ICTS'), // x8c
    NCSY = SBIG('NCSY'), // x9c
    CSSD = SBIG('CSSD'), // xa0
    IDTS = SBIG('IDTS'), // xa4
    NDSY = SBIG('NDSY'), // xb4
    IITS = SBIG('IITS'), // xb8
    PISY = SBIG('PISY'), // xc8
    SISY = SBIG('SISY'), // xcc
    KSSM = SBIG('KSSM'), // xd0
    SSWH = SBIG('SSWH'), // xd4
    SSSD = SBIG('SSSD'), // xe4
    SSPO = SBIG('SSPO'), // xe8
    SESD = SBIG('SESD'), // xf8
    SEPO = SBIG('SEPO'), // xfc
    PMLC = SBIG('PMLC'), // xec
    LTYP = SBIG('LTYP'), // x100
    LCLR = SBIG('LCLR'), // x104
    LINT = SBIG('LINT'), // x108
    LOFF = SBIG('LOFF'), // x10c
    LDIR = SBIG('LDIR'), // x110
    LFOT = SBIG('LFOT'), // x114
    LFOR = SBIG('LFOR'), // x118
    LSLA = SBIG('LSLA'), // x11c
    SELC = SBIG('SELC'), // xd8
    ORNT = SBIG('ORNT'), // x30_30
    RSOP = SBIG('RSOP'), // x30_31
    ADV1 = SBIG('ADV1'), // x10c
    ADV2 = SBIG('ADV2'), // x110
    ADV3 = SBIG('ADV3'), // x114
    ADV4 = SBIG('ADV4'), // x118
    ADV5 = SBIG('ADV5'), // x11c
    ADV6 = SBIG('ADV6'), // x120
    ADV7 = SBIG('ADV7'), // x124
    ADV8 = SBIG('ADV8'), // x128
  };
  using Properties = PPSet<
      ParticleType::GPSM,
      std::tuple<
          PP<PSIV, VectorElementFactory>,
          PP<PSVM, ModVectorElementFactory>,
          PP<PSOV, VectorElementFactory>,
          PP<PSLT, IntElementFactory>,
          PP<PSWT, IntElementFactory>,
          PP<PSTS, RealElementFactory>,
          PP<POFS, VectorElementFactory>,
          PP<SEED, IntElementFactory>,
          PP<LENG, RealElementFactory>,
          PP<WIDT, RealElementFactory>,
          PP<MAXP, IntElementFactory>,
          PP<GRTE, RealElementFactory>,
          PP<COLR, ColorElementFactory>,
          PP<LTME, IntElementFactory>,
          PP<ILOC, VectorElementFactory>,
          PP<IVEC, VectorElementFactory>,
          PP<EMTR, EmitterElementFactory>,
          PP<SORT, bool>,
          PP<MBLR, bool>,
          PP<LINE, bool>,
          PP<LIT_, bool>,
          PP<AAPH, bool>,
          PP<ZBUF, bool>,
          PP<FXLL, bool>,
          PP<PMAB, bool>,
          PP<VMD4, bool>,
          PP<VMD3, bool>,
          PP<VMD2, bool>,
          PP<VMD1, bool>,
          PP<OPTS, bool>,
          PP<PMUS, bool>,
          PP<PMOO, bool, true>,
          PP<CIND, bool>,
          PP<MBSP, IntElementFactory>,
          PP<SIZE, RealElementFactory>,
          PP<ROTA, RealElementFactory>,
          PP<TEXR, UVElementFactory<IDType>>,
          PP<TIND, UVElementFactory<IDType>>,
          PP<PMDL, ChildResourceFactory<IDType>>,
          PP<PMOP, VectorElementFactory>,
          PP<PMRT, VectorElementFactory>,
          PP<PMSC, VectorElementFactory>,
          PP<PMCL, ColorElementFactory>,
          PP<VEL1, ModVectorElementFactory>,
          PP<VEL2, ModVectorElementFactory>,
          PP<VEL3, ModVectorElementFactory>,
          PP<VEL4, ModVectorElementFactory>,
          PP<ICTS, ChildResourceFactory<IDType>>,
          PP<NCSY, IntElementFactory>,
          PP<CSSD, IntElementFactory>,
          PP<IDTS, ChildResourceFactory<IDType>>,
          PP<NDSY, IntElementFactory>,
          PP<IITS, ChildResourceFactory<IDType>>,
          PP<PISY, IntElementFactory>,
          PP<SISY, IntElementFactory>,
          PP<KSSM, SpawnSystemKeyframeData<IDType>>,
          PP<SSWH, ChildResourceFactory<IDType>>,
          PP<SSSD, IntElementFactory>,
          PP<SSPO, VectorElementFactory>,
          PP<SESD, IntElementFactory>,
          PP<SEPO, VectorElementFactory>,
          PP<PMLC, ChildResourceFactory<IDType>>,
          PP<LTYP, IntElementFactory>,
          PP<LCLR, ColorElementFactory>,
          PP<LINT, RealElementFactory>,
          PP<LOFF, VectorElementFactory>,
          PP<LDIR, VectorElementFactory>,
          PP<LFOT, IntElementFactory>,
          PP<LFOR, RealElementFactory>,
          PP<LSLA, RealElementFactory>,
          PP<SELC, ChildResourceFactory<IDType>>,
          PP<ORNT, bool>,
          PP<RSOP, bool>,
          PP<ADV1, RealElementFactory>,
          PP<ADV2, RealElementFactory>,
          PP<ADV3, RealElementFactory>,
          PP<ADV4, RealElementFactory>,
          PP<ADV5, RealElementFactory>,
          PP<ADV6, RealElementFactory>,
          PP<ADV7, RealElementFactory>,
          PP<ADV8, RealElementFactory>>,
      AAPH, ADV1, ADV2, ADV3, ADV4, ADV5, ADV6, ADV7, ADV8, CIND, COLR, CSSD, EMTR, FXLL, GRTE, ICTS, IDTS, IITS, ILOC,
      IVEC, KSSM, LCLR, LDIR, LENG, LFOR, LFOT, LINE, LINT, LIT_, LOFF, LSLA, LTME, LTYP, MAXP, MBLR, MBSP, NCSY, NDSY,
      OPTS, ORNT, PISY, PMAB, PMCL, PMDL, PMLC, PMOO, PMOP, PMRT, PMSC, PMUS, POFS, PSIV, PSLT, PSOV, PSTS, PSVM, PSWT,
      ROTA, RSOP, SEED, SELC, SEPO, SESD, SISY, SIZE, SORT, SSPO, SSSD, SSWH, TEXR, TIND, VEL1, VEL2, VEL3, VEL4, VMD1,
      VMD2, VMD3, VMD4, WIDT, ZBUF>;
};

template <class IDType>
using GPSM = PPImpl<_GPSM<IDType>>;

template <class IDType>
bool ExtractGPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteGPSM(const GPSM<IDType>& gpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
