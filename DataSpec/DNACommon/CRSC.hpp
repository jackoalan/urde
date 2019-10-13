#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "DataSpec/DNACommon/DNACommon.hpp"
#include "DataSpec/DNACommon/ParticleCommon.hpp"

#include <athena/DNA.hpp>

namespace DataSpec {
class PAKEntryReadStream;
}

namespace hecl {
class ProjectPath;
}

namespace DataSpec::DNAParticle {

enum class EWeaponCollisionResponseTypes {
  None,
  Default,
  Unknown2,
  Metal,
  Grass,
  Ice,
  Goo,
  Wood,
  Water,
  Mud,
  Lava,
  Sand,
  Projectile,
  OtherProjectile,
  Unknown14,
  Unknown15,
  EnemyNormal,
  EnemySpecial,
  EnemyShielded,
  Unknown19,
  Unknown20,
  Unknown21,
  Unknown22,
  Unknown23,
  Unknown24,
  Unknown25,
  Unknown26,
  Unknown27,
  Unknown28,
  Unknown29,
  Unknown30,
  Unknown31,
  Unknown32,
  Unknown33,
  Unknown34,
  Unknown35,
  Unknown36,
  Unknown37,
  Unknown38,
  Unknown39,
  Unknown40,
  Unknown41,
  AtomicBeta,
  AtomicAlpha,
  Unknown44,
  Unknown45,
  Unknown46,
  Unknown47,
  Unknown48,
  Unknown49,
  Unknown50,
  Unknown51,
  Unknown52,
  Unknown53,
  Unknown54,
  Unknown55,
  Unknown56,
  Unknown57,
  Unknown58,
  Unknown59,
  Unknown60,
  Unknown61,
  Unknown62,
  Unknown63,
  Unknown64,
  Unknown65,
  Unknown66,
  Unknown67,
  Unknown68,
  Unknown69,
  Unknown70,
  Unknown71,
  Unknown72,
  Unknown73,
  Unknown74,
  Unknown75,
  Unknown76,
  Unknown77,
  Unknown78,
  Unknown79,
  Unknown80,
  Unknown81,
  Unknown82,
  Unknown83,
  Unknown84,
  Unknown85,
  Unknown86,
  Unknown87,
  Unknown88,
  Unknown89,
  Unknown90,
  Unknown91,
  AtomicBetaReflect,
  AtomicAlphaReflect
};

struct ECPGenerator {
  template <typename IDType>
  using Type = ChildResourceFactory<IDType>;
};
struct ECPDecal {
  template <typename IDType>
  using Type = ChildResourceFactory<IDType>;
};
struct ECPSfx {
  template <typename IDType>
  using Type = ValueHelper<uint32_t>;
};

class CPUtils {
  template<std::size_t _Count, typename _Tuple>
  struct _CountCPs { static constexpr std::size_t Count = _Count; };

  template<std::size_t _Count, typename _First, typename... _Rest>
  struct _CountCPs<_Count, std::tuple<_First, _Rest...>>
      : _CountCPs<_Count + (_First::IsCP ? 1 : 0), std::tuple<_Rest...>> {};

  template<bool _IsCP, std::size_t _Idx, typename _Element, typename _ExistingTuple>
  struct _AppendElement;

  template<std::size_t _Idx, typename _Element, typename... _Existing>
  struct _AppendElement<true, _Idx, _Element, std::tuple<_Existing...>> {
    using Tuple = typename _Element::template Expand<_Idx, _Existing...>::Tuple;
  };

  template<std::size_t _Idx, typename _Element, typename... _Existing>
  struct _AppendElement<false, _Idx, _Element, std::tuple<_Existing...>> {
    using Tuple = std::tuple<_Existing..., _Element>;
  };

  template<std::size_t _Idx, typename _OutTuple, typename _Tuple>
  struct _ExpandTuple { using Tuple = _OutTuple; };

  template<std::size_t _Idx, typename _OutTuple, typename _First, typename... _Rest>
  struct _ExpandTuple<_Idx, _OutTuple, std::tuple<_First, _Rest...>>
      : _ExpandTuple<_Idx + 1, typename _AppendElement<_First::IsCP, _Idx, _First, _OutTuple>::Tuple,
                     std::tuple<_Rest...>> {};
public:
  template<typename _Props>
  using CountCPs = _CountCPs<0, typename _Props::RawPropsTuple>;
  template<typename _PropsTuple>
  using ExpandTuple = _ExpandTuple<0, std::tuple<>, _PropsTuple>;
};

template <auto FCC, typename _SubProp, typename _IDType, std::size_t _Index>
struct CP {
  static constexpr bool IsBool = false;
  using SubProp = _SubProp;
  using Type = typename _SubProp::template Type<_IDType>;
  using FourCC = std::integral_constant<decltype(FCC), FCC>;
  static constexpr std::size_t Index = _Index;
  template<typename _Props>
  Type& GetRef(_Props& p) { return p.GetArray(SubProp{})[Index]; }
  template<typename _Props>
  bool ShouldStore(_Props& p) { return GetRef(p).operator bool(); }
};

template <auto _GenFCC, auto _DecalFCC, auto _SfxFCC, typename _IDType>
struct CPS {
  static constexpr bool IsBool = false;
  static constexpr bool IsCP = true;
  using GenFCC = std::integral_constant<decltype(_GenFCC), _GenFCC>;
  using DecalFCC = std::integral_constant<decltype(_DecalFCC), _DecalFCC>;
  using SfxFCC = std::integral_constant<decltype(_SfxFCC), _SfxFCC>;
  struct _ExpandDone {
    template<std::size_t Index, typename... Existing>
    using Tuple = std::tuple<Existing...>;
  };
  template<typename FCC, typename SubProp, typename Next>
  struct _Expand {
    template<std::size_t Index, typename... Existing>
    using Tuple =
      std::conditional_t<FCC::value != typename FCC::value_type(0),
      typename Next::template Tuple<Index, Existing..., CP<FCC::value, SubProp, _IDType, Index>>,
      typename Next::template Tuple<Index, Existing...>>;
  };
  template<std::size_t Index, typename... Existing>
  struct Expand {
    using Tuple =
    typename _Expand<GenFCC, ECPGenerator,
             _Expand<DecalFCC, ECPDecal,
             _Expand<SfxFCC, ECPSfx,
             _ExpandDone>>>::template Tuple<Index, Existing...>;
  };
};

template <ParticleType _Type, typename _IDType, typename _PropsTuple, auto... _SortedTypes>
struct CPSet : _PPSet<typename CPUtils::ExpandTuple<_PropsTuple>::Tuple, _SortedTypes...> {
  static constexpr ParticleType Type = _Type;
  using Self = CPSet<_Type, _IDType, _PropsTuple, _SortedTypes...>;
  using Base = _PPSet<typename CPUtils::ExpandTuple<_PropsTuple>::Tuple, _SortedTypes...>;
  using RawPropsTuple = _PropsTuple;
  static constexpr std::size_t ArraySize = CPUtils::template CountCPs<Self>::Count;
  std::array<ECPGenerator::Type<_IDType>, ArraySize> m_gens;
  std::array<ECPDecal::Type<_IDType>, ArraySize> m_decals;
  std::array<ECPSfx::Type<_IDType>, ArraySize> m_sfxs;
  typename Base::PropsTuple m_props;
  auto& GetArray(ECPGenerator) { return m_gens; }
  auto& GetArray(ECPDecal) { return m_decals; }
  auto& GetArray(ECPSfx) { return m_sfxs; }
};

template <typename _Basis>
struct CPImpl : PPImpl<_Basis> {
  using Base = PPImpl<_Basis>;
  auto getGeneratorId(EWeaponCollisionResponseTypes type) const {
    return Base::m_props.m_gens[std::size_t(type)].id;
  }
  auto getDecalId(EWeaponCollisionResponseTypes type) const {
    return Base::m_props.m_decals[std::size_t(type)].id;
  }
  auto getSfx(EWeaponCollisionResponseTypes type) const {
    return Base::m_props.m_sfxs[std::size_t(type)].value;
  }
};

template <class IDType>
struct _CRSM {
  enum EPP : uint32_t {
    NONE = 0,

    NODP = SBIG('NODP'),
    DEFS = SBIG('DEFS'),
    CRTS = SBIG('CRTS'),
    MTLS = SBIG('MTLS'),
    GRAS = SBIG('GRAS'),
    ICEE = SBIG('ICEE'),
    GOOO = SBIG('GOOO'),
    WODS = SBIG('WODS'),
    WATR = SBIG('WATR'),
    _1MUD = SBIG('1MUD'),
    _1LAV = SBIG('1LAV'),
    _1SAN = SBIG('1SAN'),
    _1PRJ = SBIG('1PRJ'),
    DCHR = SBIG('DCHR'),
    DCHS = SBIG('DCHS'),
    DCSH = SBIG('DCSH'),
    DENM = SBIG('DENM'),
    DESP = SBIG('DESP'),
    DESH = SBIG('DESH'),
    BTLE = SBIG('BTLE'),
    WASP = SBIG('WASP'),
    TALP = SBIG('TALP'),
    PTGM = SBIG('PTGM'),
    SPIR = SBIG('SPIR'),
    FPIR = SBIG('FPIR'),
    FFLE = SBIG('FFLE'),
    PARA = SBIG('PARA'),
    BMON = SBIG('BMON'),
    BFLR = SBIG('BFLR'),
    PBOS = SBIG('PBOS'),
    IBOS = SBIG('IBOS'),
    _1SVA = SBIG('1SVA'),
    _1RPR = SBIG('1RPR'),
    _1MTR = SBIG('1MTR'),
    _1PDS = SBIG('1PDS'),
    _1FLB = SBIG('1FLB'),
    _1DRN = SBIG('1DRN'),
    _1MRE = SBIG('1MRE'),
    CHOZ = SBIG('CHOZ'),
    JZAP = SBIG('JZAP'),
    _1ISE = SBIG('1ISE'),
    _1BSE = SBIG('1BSE'),
    _1ATB = SBIG('1ATB'),
    _1ATA = SBIG('1ATA'),
    BTSP = SBIG('BTSP'),
    WWSP = SBIG('WWSP'),
    TASP = SBIG('TASP'),
    TGSP = SBIG('TGSP'),
    SPSP = SBIG('SPSP'),
    FPSP = SBIG('FPSP'),
    FFSP = SBIG('FFSP'),
    PSSP = SBIG('PSSP'),
    BMSP = SBIG('BMSP'),
    BFSP = SBIG('BFSP'),
    PBSP = SBIG('PBSP'),
    IBSP = SBIG('IBSP'),
    _2SVA = SBIG('2SVA'),
    _2RPR = SBIG('2RPR'),
    _2MTR = SBIG('2MTR'),
    _2PDS = SBIG('2PDS'),
    _2FLB = SBIG('2FLB'),
    _2DRN = SBIG('2DRN'),
    _2MRE = SBIG('2MRE'),
    CHSP = SBIG('CHSP'),
    JZSP = SBIG('JZSP'),
    _3ISE = SBIG('3ISE'),
    _3BSE = SBIG('3BSE'),
    _3ATB = SBIG('3ATB'),
    _3ATA = SBIG('3ATA'),
    BTSH = SBIG('BTSH'),
    WWSH = SBIG('WWSH'),
    TASH = SBIG('TASH'),
    TGSH = SBIG('TGSH'),
    SPSH = SBIG('SPSH'),
    FPSH = SBIG('FPSH'),
    FFSH = SBIG('FFSH'),
    PSSH = SBIG('PSSH'),
    BMSH = SBIG('BMSH'),
    BFSH = SBIG('BFSH'),
    PBSH = SBIG('PBSH'),
    IBSH = SBIG('IBSH'),
    _3SVA = SBIG('3SVA'),
    _3RPR = SBIG('3RPR'),
    _3MTR = SBIG('3MTR'),
    _3PDS = SBIG('3PDS'),
    _3FLB = SBIG('3FLB'),
    _3DRN = SBIG('3DRN'),
    _3MRE = SBIG('3MRE'),
    CHSH = SBIG('CHSH'),
    JZSH = SBIG('JZSH'),
    _5ISE = SBIG('5ISE'),
    _5BSE = SBIG('5BSE'),
    _5ATB = SBIG('5ATB'),
    _5ATA = SBIG('5ATA'),

    NSFX = SBIG('NSFX'),
    DSFX = SBIG('DSFX'),
    CSFX = SBIG('CSFX'),
    MSFX = SBIG('MSFX'),
    GRFX = SBIG('GRFX'),
    ICFX = SBIG('ICFX'),
    GOFX = SBIG('GOFX'),
    WSFX = SBIG('WSFX'),
    WTFX = SBIG('WTFX'),
    _2MUD = SBIG('2MUD'),
    _2LAV = SBIG('2LAV'),
    _2SAN = SBIG('2SAN'),
    _2PRJ = SBIG('2PRJ'),
    DCFX = SBIG('DCFX'),
    DSHX = SBIG('DSHX'),
    DEFX = SBIG('DEFX'),
    ESFX = SBIG('ESFX'),
    SHFX = SBIG('SHFX'),
    BEFX = SBIG('BEFX'),
    WWFX = SBIG('WWFX'),
    TAFX = SBIG('TAFX'),
    GTFX = SBIG('GTFX'),
    SPFX = SBIG('SPFX'),
    FPFX = SBIG('FPFX'),
    FFFX = SBIG('FFFX'),
    PAFX = SBIG('PAFX'),
    BMFX = SBIG('BMFX'),
    BFFX = SBIG('BFFX'),
    PBFX = SBIG('PBFX'),
    IBFX = SBIG('IBFX'),
    _4SVA = SBIG('4SVA'),
    _4RPR = SBIG('4RPR'),
    _4MTR = SBIG('4MTR'),
    _4PDS = SBIG('4PDS'),
    _4FLB = SBIG('4FLB'),
    _4DRN = SBIG('4DRN'),
    _4MRE = SBIG('4MRE'),
    CZFX = SBIG('CZFX'),
    JZAS = SBIG('JZAS'),
    _2ISE = SBIG('2ISE'),
    _2BSE = SBIG('2BSE'),
    _2ATB = SBIG('2ATB'),
    _2ATA = SBIG('2ATA'),
    BSFX = SBIG('BSFX'),
    TSFX = SBIG('TSFX'),
    GSFX = SBIG('GSFX'),
    SSFX = SBIG('SSFX'),
    FSFX = SBIG('FSFX'),
    SFFX = SBIG('SFFX'),
    PSFX = SBIG('PSFX'),
    SBFX = SBIG('SBFX'),
    PBSX = SBIG('PBSX'),
    IBSX = SBIG('IBSX'),
    _5SVA = SBIG('5SVA'),
    _5RPR = SBIG('5RPR'),
    _5MTR = SBIG('5MTR'),
    _5PDS = SBIG('5PDS'),
    _5FLB = SBIG('5FLB'),
    _5DRN = SBIG('5DRN'),
    _5MRE = SBIG('5MRE'),
    JZPS = SBIG('JZPS'),
    _4ISE = SBIG('4ISE'),
    _4BSE = SBIG('4BSE'),
    _4ATB = SBIG('4ATB'),
    _4ATA = SBIG('4ATA'),
    BHFX = SBIG('BHFX'),
    WHFX = SBIG('WHFX'),
    THFX = SBIG('THFX'),
    GHFX = SBIG('GHFX'),
    FHFX = SBIG('FHFX'),
    HFFX = SBIG('HFFX'),
    PHFX = SBIG('PHFX'),
    MHFX = SBIG('MHFX'),
    HBFX = SBIG('HBFX'),
    PBHX = SBIG('PBHX'),
    IBHX = SBIG('IBHX'),
    _6SVA = SBIG('6SVA'),
    _6RPR = SBIG('6RPR'),
    _6MTR = SBIG('6MTR'),
    _6PDS = SBIG('6PDS'),
    _6FLB = SBIG('6FLB'),
    _6DRN = SBIG('6DRN'),
    _6MRE = SBIG('6MRE'),
    CHFX = SBIG('CHFX'),
    JZHS = SBIG('JZHS'),
    _6ISE = SBIG('6ISE'),
    _6BSE = SBIG('6BSE'),
    _6ATB = SBIG('6ATB'),
    _6ATA = SBIG('6ATA'),

    NCDL = SBIG('NCDL'),
    DDCL = SBIG('DDCL'),
    CODL = SBIG('CODL'),
    MEDL = SBIG('MEDL'),
    GRDL = SBIG('GRDL'),
    ICDL = SBIG('ICDL'),
    GODL = SBIG('GODL'),
    WODL = SBIG('WODL'),
    WTDL = SBIG('WTDL'),
    _3MUD = SBIG('3MUD'),
    _3LAV = SBIG('3LAV'),
    _3SAN = SBIG('3SAN'),
    CHDL = SBIG('CHDL'),
    ENDL = SBIG('ENDL'),

    RNGE = SBIG('RNGE'), // x30
    FOFF = SBIG('FOFF'), // x34
  };

  using Properties = CPSet<
      ParticleType::CRSM, IDType,
      std::tuple<
          CPS<NODP, NCDL, NSFX, IDType>,
          CPS<DEFS, DDCL, DSFX, IDType>,
          CPS<CRTS, CODL, CSFX, IDType>,
          CPS<MTLS, MEDL, MSFX, IDType>,
          CPS<GRAS, GRDL, GRFX, IDType>,
          CPS<ICEE, ICDL, ICFX, IDType>,
          CPS<GOOO, GODL, GOFX, IDType>,
          CPS<WODS, WODL, WSFX, IDType>,
          CPS<WATR, WTDL, WTFX, IDType>,
          CPS<_1MUD, _3MUD, _2MUD, IDType>,
          CPS<_1LAV, _3LAV, _2LAV, IDType>,
          CPS<_1SAN, _3SAN, _2SAN, IDType>,
          CPS<_1PRJ, CHDL, _2PRJ, IDType>,
          CPS<DCHR, ENDL, DCFX, IDType>,
          CPS<DCHS, NONE, NONE, IDType>,
          CPS<DCSH, NONE, DSHX, IDType>,
          CPS<DENM, NONE, DEFX, IDType>,
          CPS<DESP, NONE, ESFX, IDType>,
          CPS<DESH, NONE, SHFX, IDType>,
          CPS<BTLE, NONE, BEFX, IDType>,
          CPS<WASP, NONE, WWFX, IDType>,
          CPS<TALP, NONE, TAFX, IDType>,
          CPS<PTGM, NONE, GTFX, IDType>,
          CPS<SPIR, NONE, SPFX, IDType>,
          CPS<FPIR, NONE, FPFX, IDType>,
          CPS<FFLE, NONE, FFFX, IDType>,
          CPS<PARA, NONE, PAFX, IDType>,
          CPS<BMON, NONE, BMFX, IDType>,
          CPS<BFLR, NONE, BFFX, IDType>,
          CPS<PBOS, NONE, PBFX, IDType>,
          CPS<IBOS, NONE, IBFX, IDType>,
          CPS<_1SVA, NONE, _4SVA, IDType>,
          CPS<_1RPR, NONE, _4RPR, IDType>,
          CPS<_1MTR, NONE, _4MTR, IDType>,
          CPS<_1PDS, NONE, _4PDS, IDType>,
          CPS<_1FLB, NONE, _4FLB, IDType>,
          CPS<_1DRN, NONE, _4DRN, IDType>,
          CPS<_1MRE, NONE, _4MRE, IDType>,
          CPS<CHOZ, NONE, CZFX, IDType>,
          CPS<JZAP, NONE, JZAS, IDType>,
          CPS<_1ISE, NONE, _2ISE, IDType>,
          CPS<_1BSE, NONE, _2BSE, IDType>,
          CPS<_1ATB, NONE, _2ATB, IDType>,
          CPS<_1ATA, NONE, _2ATA, IDType>,
          CPS<BTSP, NONE, BSFX, IDType>,
          CPS<WWSP, NONE, NONE, IDType>,
          CPS<TASP, NONE, TSFX, IDType>,
          CPS<TGSP, NONE, GSFX, IDType>,
          CPS<SPSP, NONE, SSFX, IDType>,
          CPS<FPSP, NONE, FSFX, IDType>,
          CPS<FFSP, NONE, SFFX, IDType>,
          CPS<PSSP, NONE, PSFX, IDType>,
          CPS<BMSP, NONE, NONE, IDType>,
          CPS<BFSP, NONE, SBFX, IDType>,
          CPS<PBSP, NONE, PBSX, IDType>,
          CPS<IBSP, NONE, IBSX, IDType>,
          CPS<_2SVA, NONE, _5SVA, IDType>,
          CPS<_2RPR, NONE, _5RPR, IDType>,
          CPS<_2MTR, NONE, _5MTR, IDType>,
          CPS<_2PDS, NONE, _5PDS, IDType>,
          CPS<_2FLB, NONE, _5FLB, IDType>,
          CPS<_2DRN, NONE, _5DRN, IDType>,
          CPS<_2MRE, NONE, _5MRE, IDType>,
          CPS<CHSP, NONE, NONE, IDType>,
          CPS<JZSP, NONE, JZPS, IDType>,
          CPS<_3ISE, NONE, _4ISE, IDType>,
          CPS<_3BSE, NONE, _4BSE, IDType>,
          CPS<_3ATB, NONE, _4ATB, IDType>,
          CPS<_3ATA, NONE, _4ATA, IDType>,
          CPS<BTSH, NONE, BHFX, IDType>,
          CPS<WWSH, NONE, WHFX, IDType>,
          CPS<TASH, NONE, THFX, IDType>,
          CPS<TGSH, NONE, GHFX, IDType>,
          CPS<SPSH, NONE, NONE, IDType>,
          CPS<FPSH, NONE, FHFX, IDType>,
          CPS<FFSH, NONE, HFFX, IDType>,
          CPS<PSSH, NONE, PHFX, IDType>,
          CPS<BMSH, NONE, MHFX, IDType>,
          CPS<BFSH, NONE, HBFX, IDType>,
          CPS<PBSH, NONE, PBHX, IDType>,
          CPS<IBSH, NONE, IBHX, IDType>,
          CPS<_3SVA, NONE, _6SVA, IDType>,
          CPS<_3RPR, NONE, _6RPR, IDType>,
          CPS<_3MTR, NONE, _6MTR, IDType>,
          CPS<_3PDS, NONE, _6PDS, IDType>,
          CPS<_3FLB, NONE, _6FLB, IDType>,
          CPS<_3DRN, NONE, _6DRN, IDType>,
          CPS<_3MRE, NONE, _6MRE, IDType>,
          CPS<CHSH, NONE, CHFX, IDType>,
          CPS<JZSH, NONE, JZHS, IDType>,
          CPS<_5ISE, NONE, _6ISE, IDType>,
          CPS<_5BSE, NONE, _6BSE, IDType>,
          CPS<_5ATB, NONE, _6ATB, IDType>,
          CPS<_5ATA, NONE, _6ATA, IDType>,
          PP<RNGE, ValueHelper<float>>,
          PP<FOFF, ValueHelper<float>>>,
      _1ATA, _1ATB, _1BSE, _1DRN, _1FLB, _1ISE, _1LAV, _1MRE, _1MTR, _1MUD, _1PDS, _1PRJ, _1RPR, _1SAN, _1SVA, _2ATA,
      _2ATB, _2BSE, _2DRN, _2FLB, _2ISE, _2LAV, _2MRE, _2MTR, _2MUD, _2PDS, _2PRJ, _2RPR, _2SAN, _2SVA, _3ATA, _3ATB,
      _3BSE, _3DRN, _3FLB, _3ISE, _3LAV, _3MRE, _3MTR, _3MUD, _3PDS, _3RPR, _3SAN, _3SVA, _4ATA, _4ATB, _4BSE, _4DRN,
      _4FLB, _4ISE, _4MRE, _4MTR, _4PDS, _4RPR, _4SVA, _5ATA, _5ATB, _5BSE, _5DRN, _5FLB, _5ISE, _5MRE, _5MTR, _5PDS,
      _5RPR, _5SVA, _6ATA, _6ATB, _6BSE, _6DRN, _6FLB, _6ISE, _6MRE, _6MTR, _6PDS, _6RPR, _6SVA, BEFX, BFFX, BFLR, BFSH,
      BFSP, BHFX, BMFX, BMON, BMSH, BMSP, BSFX, BTLE, BTSH, BTSP, CHDL, CHFX, CHOZ, CHSH, CHSP, CODL, CRTS, CSFX, CZFX,
      DCFX, DCHR, DCHS, DCSH, DDCL, DEFS, DEFX, DENM, DESH, DESP, DSFX, DSHX, ENDL, ESFX, FFFX, FFLE, FFSH, FFSP, FHFX,
      FOFF, FPFX, FPIR, FPSH, FPSP, FSFX, GHFX, GODL, GOFX, GOOO, GRAS, GRDL, GRFX, GSFX, GTFX, HBFX, HFFX, IBFX, IBHX,
      IBOS, IBSH, IBSP, IBSX, ICDL, ICEE, ICFX, JZAP, JZAS, JZHS, JZPS, JZSH, JZSP, MEDL, MHFX, MSFX, MTLS, NCDL, NODP,
      NSFX, PAFX, PARA, PBFX, PBHX, PBOS, PBSH, PBSP, PBSX, PHFX, PSFX, PSSH, PSSP, PTGM, RNGE, SBFX, SFFX, SHFX, SPFX,
      SPIR, SPSH, SPSP, SSFX, TAFX, TALP, TASH, TASP, TGSH, TGSP, THFX, TSFX, WASP, WATR, WHFX, WODL, WODS, WSFX, WTDL,
      WTFX, WWFX, WWSH, WWSP>;
};
template <class IDType>
using CRSM = CPImpl<_CRSM<IDType>>;

template <class IDType>
bool ExtractCRSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteCRSM(const CRSM<IDType>& crsm, const hecl::ProjectPath& outPath);
} // namespace DataSpec::DNAParticle
