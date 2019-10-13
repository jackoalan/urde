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
  enum EPP : uint32_t {
    IORN = SBIG('IORN'), // x0
    IVEC = SBIG('IVEC'), // x4
    PSOV = SBIG('PSOV'), // x8
    PSVM = SBIG('PSVM'), // xc
    VMD2 = SBIG('VMD2'), // x10
    PSLT = SBIG('PSLT'), // x14
    PSCL = SBIG('PSCL'), // x18
    PCOL = SBIG('PCOL'), // x1c
    POFS = SBIG('POFS'), // x20
    OFST = SBIG('OFST'), // x24
    APSO = SBIG('APSO'), // x28
    HOMG = SBIG('HOMG'), // x29
    AP11 = SBIG('AP11'), // x2a
    AP21 = SBIG('AP21'), // x2b
    AS11 = SBIG('AS11'), // x2c
    AS12 = SBIG('AS12'), // x2d
    AS13 = SBIG('AS13'), // x2e
    TRAT = SBIG('TRAT'), // x30
    APSM = SBIG('APSM'), // x34
    APS2 = SBIG('APS2'), // x44
    ASW1 = SBIG('ASW1'), // x54
    ASW2 = SBIG('ASW2'), // x64
    ASW3 = SBIG('ASW3'), // x74
    OHEF = SBIG('OHEF'), // x84
    COLR = SBIG('COLR'), // x94
    EWTR = SBIG('EWTR'), // xa4
    LWTR = SBIG('LWTR'), // xa5
    SWTR = SBIG('SWTR'), // xa6
    PJFX = SBIG('PJFX'), // xa8
    RNGE = SBIG('RNGE'), // xac
    FOFF = SBIG('FOFF'), // xb0
    FC60 = SBIG('FC60'), // xunk
    SPS1 = SBIG('SPS1'), // xunk
    SPS2 = SBIG('SPS2'), // xunk
  };
  using Properties = PPSet<
      ParticleType::WPSM,
      std::tuple<
          PP<IORN, VectorElementFactory>,
          PP<IVEC, VectorElementFactory>,
          PP<PSOV, VectorElementFactory>,
          PP<PSVM, ModVectorElementFactory>,
          PP<VMD2, bool>,
          PP<PSLT, IntElementFactory>,
          PP<PSCL, VectorElementFactory>,
          PP<PCOL, ColorElementFactory>,
          PP<POFS, VectorElementFactory>,
          PP<OFST, VectorElementFactory>,
          PP<APSO, bool>,
          PP<HOMG, bool>,
          PP<AP11, bool>,
          PP<AP21, bool>,
          PP<AS11, bool>,
          PP<AS12, bool>,
          PP<AS13, bool>,
          PP<TRAT, RealElementFactory>,
          PP<APSM, ChildResourceFactory<IDType>>,
          PP<APS2, ChildResourceFactory<IDType>>,
          PP<ASW1, ChildResourceFactory<IDType>>,
          PP<ASW2, ChildResourceFactory<IDType>>,
          PP<ASW3, ChildResourceFactory<IDType>>,
          PP<OHEF, ChildResourceFactory<IDType>>,
          PP<COLR, ChildResourceFactory<IDType>>,
          PP<EWTR, bool, true>,
          PP<LWTR, bool, true>,
          PP<SWTR, bool, true>,
          PP<PJFX, ValueHelper<uint32_t>>,
          PP<RNGE, RealElementFactory>,
          PP<FOFF, RealElementFactory>,
          PP<FC60, bool>,
          PP<SPS1, bool>,
          PP<SPS2, bool>>,
      AP11, AP21, APS2, APSM, APSO, AS11, AS12, AS13, ASW1, ASW2, ASW3, COLR, EWTR, FC60, FOFF, HOMG, IORN, IVEC,
      LWTR, OFST, OHEF, PCOL, PJFX, POFS, PSCL, PSLT, PSOV, PSVM, RNGE, SPS1, SPS2, SWTR, TRAT, VMD2>;
};

template <class IDType>
using WPSM = PPImpl<_WPSM<IDType>>;

template <class IDType>
bool ExtractWPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteWPSM(const WPSM<IDType>& wpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
