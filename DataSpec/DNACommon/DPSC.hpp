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

class DPUtils {
  template<std::size_t _Idx, typename _Key, typename _Tuple>
  struct _Match;

  template<std::size_t _Idx, typename _Key, typename _First, typename... _Rest>
  struct _Match<_Idx, _Key, std::tuple<_First, _Rest...>>
      : _Match<_Idx + 1, _Key, std::tuple<_Rest...>> {};

  template<std::size_t _Idx, typename _First, typename... _Rest>
  struct _Match<_Idx, typename _First::FourCC, std::tuple<_First, _Rest...>>
  { using Type = _First; static constexpr bool IsDP = _First::IsDP; };

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
      : _ExpandTuple<_Idx + 1, typename _AppendElement<_First::IsDP, _Idx, _First, _OutTuple>::Tuple,
          std::tuple<_Rest...>> {};
public:
  template<auto _Key, typename _Props>
  using GetElement = _Match<0, std::integral_constant<decltype(_Key), _Key>, typename _Props::PropsTuple>;
  template<typename _PropsTuple>
  using ExpandTuple = _ExpandTuple<0, std::tuple<>, _PropsTuple>;

  template<auto _Key, typename _Props>
  static constexpr decltype(auto) GetQuad(_Props& p, std::size_t idx) {
    using Elem = typename GetElement<_Key, _Props>::Type;
    static_assert(Elem::IsDP, "GetQuad may only be used to look up quad properties");
    if constexpr (Elem::IsDP) {
      if (idx)
        return PPUtils::Get<Elem::Type::FCC2::value>(p);
      else
        return PPUtils::Get<Elem::Type::FCC1::value>(p);
    }
  }
};

template <auto _FCC, auto _FCC1, auto _FCC2, typename _Type>
struct DPS {
  static constexpr bool IsBool = false;
  static constexpr bool IsDP = true;
  using FourCC = std::integral_constant<decltype(_FCC), _FCC>;
  using FCC1 = std::integral_constant<decltype(_FCC1), _FCC1>;
  using FCC2 = std::integral_constant<decltype(_FCC2), _FCC2>;
  struct _ExpandDone {
    template<std::size_t Index, typename... Existing>
    using Tuple = std::tuple<Existing...>;
  };
  template<typename FCC, typename Next>
  struct _Expand {
    template<std::size_t Index, typename... Existing>
    using Tuple =
    std::conditional_t<FCC::value != typename FCC::value_type(0),
        typename Next::template Tuple<Index, Existing..., PP<FCC::value, _Type>>,
        typename Next::template Tuple<Index, Existing...>>;
  };
  template<std::size_t Index, typename... Existing>
  struct Expand {
    using Tuple = typename _Expand<FCC1, _Expand<FCC2, _ExpandDone>>::template Tuple<Index, Existing...>;
  };
};

template <ParticleType _Type, typename _PropsTuple, auto... _SortedTypes>
struct DPSet : _PPSet<typename DPUtils::ExpandTuple<_PropsTuple>::Tuple, _SortedTypes...> {
  static constexpr ParticleType Type = _Type;
  using Self = DPSet<_Type, _PropsTuple, _SortedTypes...>;
  using Base = _PPSet<typename DPUtils::ExpandTuple<_PropsTuple>::Tuple, _SortedTypes...>;
  using RawPropsTuple = _PropsTuple;
  typename Base::PropsTuple m_props;
  typename Base::PropsBitset m_boolProps;
  DPSet() {
    std::apply([&](auto&&... p) {
      ([&](auto&& p) {
        using Tp = std::decay_t<decltype(p)>;
        if constexpr (Tp::IsBool)
          if constexpr (Tp::DefaultBool::value)
            m_boolProps[PPUtils::GetStoreIndex<Tp, Base>::Index] = true;
      }(p), ...);
    }, m_props);
  }
};

template <typename _Basis>
struct DPImpl : PPImpl<_Basis> {
  using Base = PPImpl<_Basis>;
  using EDP = typename _Basis::EDP;
  template<EDP _Key>
  decltype(auto) getQuad(std::size_t idx) { return DPUtils::GetQuad<_Key>(Base::m_props, idx); }
};

template <class IDType>
struct _DPSM {
  enum EPP : uint32_t {
    _1LFT = SBIG('1LFT'), // x0
    _1SZE = SBIG('1SZE'), // x4
    _1ROT = SBIG('1ROT'), // x8
    _1OFF = SBIG('1OFF'), // xc
    _1CLR = SBIG('1CLR'), // x10
    _1TEX = SBIG('1TEX'), // x14
    _1ADD = SBIG('1ADD'), // x18
    _2LFT = SBIG('2LFT'), // x1c
    _2SZE = SBIG('2SZE'), // x20
    _2ROT = SBIG('2ROT'), // x24
    _2OFF = SBIG('2OFF'), // x28
    _2CLR = SBIG('2CLR'), // x2c
    _2TEX = SBIG('2TEX'), // x30
    _2ADD = SBIG('2ADD'), // x34
    DMDL = SBIG('DMDL'), // x38
    DLFT = SBIG('DLFT'), // x48
    DMOP = SBIG('DMOP'), // x4c
    DMRT = SBIG('DMRT'), // x50
    DMSC = SBIG('DMSC'), // x54
    DMCL = SBIG('DMCL'), // x58
    DMAB = SBIG('DMAB'), // x5c_24
    DMOO = SBIG('DMOO'), // x5c_25
  };
  enum EDP : uint32_t {
    LFT = SBIG(' LFT'), // x0
    SZE = SBIG(' SZE'), // x4
    ROT = SBIG(' ROT'), // x8
    OFF = SBIG(' OFF'), // xc
    CLR = SBIG(' CLR'), // x10
    TEX = SBIG(' TEX'), // x14
    ADD = SBIG(' ADD'), // x18
  };
  using Properties = DPSet<
      ParticleType::DPSM,
      std::tuple<
          DPS<LFT, _1LFT, _2LFT, IntElementFactory>,
          DPS<SZE, _1SZE, _2SZE, RealElementFactory>,
          DPS<ROT, _1ROT, _2ROT, RealElementFactory>,
          DPS<OFF, _1OFF, _2OFF, VectorElementFactory>,
          DPS<CLR, _1CLR, _2CLR, ColorElementFactory>,
          DPS<TEX, _1TEX, _2TEX, UVElementFactory<IDType>>,
          DPS<ADD, _1ADD, _2ADD, bool>,
          PP<DMDL, ChildResourceFactory<IDType>>,
          PP<DLFT, IntElementFactory>,
          PP<DMOP, VectorElementFactory>,
          PP<DMRT, VectorElementFactory>,
          PP<DMSC, VectorElementFactory>,
          PP<DMCL, ColorElementFactory>,
          PP<DMAB, bool>,
          PP<DMOO, bool>>,
      _1ADD, _1CLR, _1LFT, _1OFF, _1ROT, _1SZE, _1TEX, _2ADD, _2CLR, _2LFT, _2OFF, _2ROT, _2SZE, _2TEX, DLFT,
      DMAB, DMCL, DMDL, DMOO, DMOP, DMRT, DMSC>;
};

template <class IDType>
using DPSM = DPImpl<_DPSM<IDType>>;

template <class IDType>
bool ExtractDPSM(PAKEntryReadStream& rs, const hecl::ProjectPath& outPath);

template <class IDType>
bool WriteDPSM(const DPSM<IDType>& dpsm, const hecl::ProjectPath& outPath);

} // namespace DataSpec::DNAParticle
