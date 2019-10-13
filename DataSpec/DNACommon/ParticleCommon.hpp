#pragma once

#include <memory>
#include <vector>

#include "DataSpec/DNACommon/DNACommon.hpp"

#include <logvisor/logvisor.hpp>

#if defined(__GNUC__) && !defined(__clang__)
#warning GCC compiles this template system very slowly. Using clang is highly recommended.
#endif

namespace DataSpec::DNAParticle {
extern logvisor::Module LogModule;

enum class ParticleType {
  GPSM = SBIG('GPSM'),
  SWSH = SBIG('SWSH'),
  ELSM = SBIG('ELSM'),
  DPSM = SBIG('DPSM'),
  CRSM = SBIG('CRSM'),
  WPSM = SBIG('WPSM')
};

/*
 * The particle property (PP) metaclass system provides common compile-time utilities
 * for storing, enumerating, and streaming particle scripts.
 */

class PPUtils {
  friend class PEUtils;

  template<std::size_t _Idx, typename _Key, typename _Tuple>
  struct _Match;

  template<std::size_t _Idx, typename _Key, typename _First, typename... _Rest>
  struct _Match<_Idx, _Key, std::tuple<_First, _Rest...>>
      : _Match<_Idx + 1, _Key, std::tuple<_Rest...>> {};

  template<std::size_t _Idx, typename _First, typename... _Rest>
  struct _Match<_Idx, typename _First::FourCC, std::tuple<_First, _Rest...>>
  { using Type = _First; static constexpr std::size_t Index = _Idx; };

  template<std::size_t _Idx, std::size_t _BIdx, typename _Tuple>
  struct _Count { static constexpr std::size_t Count = _Idx; static constexpr std::size_t BoolCount = _BIdx; };

  template<std::size_t _Idx, std::size_t _BIdx, typename _First, typename... _Rest>
  struct _Count<_Idx, _BIdx, std::tuple<_First, _Rest...>>
      : _Count<_Idx + 1, _BIdx + (_First::IsBool ? 1 : 0), std::tuple<_Rest...>> {};

  template<std::size_t _Idx, std::size_t _BIdx, typename _Type, typename _Tuple>
  struct _GetStoreIndex;

  template<std::size_t _Idx, std::size_t _BIdx, typename _Type, typename _First, typename... _Rest>
  struct _GetStoreIndex<_Idx, _BIdx, _Type, std::tuple<_First, _Rest...>>
      : _GetStoreIndex<_Idx + 1, _BIdx + (_First::IsBool ? 1 : 0),
          _Type, std::tuple<_Rest...>> {};

  template<std::size_t _Idx, std::size_t _BIdx, typename _First, typename... _Rest>
  struct _GetStoreIndex<_Idx, _BIdx, _First, std::tuple<_First, _Rest...>> {
    static constexpr bool IsBool = _First::IsBool;
    static constexpr std::size_t Index = (_First::IsBool ? _BIdx : _Idx);
  };

  template<typename _PropsTuple, typename _Indices>
  struct _Validate {};

  template<typename _PropsTuple, std::size_t _First, std::size_t _Second, std::size_t... _Rest>
  struct _Validate<_PropsTuple, std::index_sequence<_First, _Second, _Rest...>>
      : _Validate<_PropsTuple, std::index_sequence<_Second, _Rest...>> {
    static constexpr auto First = std::tuple_element_t<_First, _PropsTuple>::FourCC::value;
    static constexpr auto Second = std::tuple_element_t<_Second, _PropsTuple>::FourCC::value;
    static_assert(SBIG(First) < SBIG(Second), "Property set is not sorted.");
  };

  template<auto _Idx, typename _Props>
  struct _PropGetFunc {
    using _Tp = std::tuple_element_t<_Idx, typename _Props::PropsTuple>;
    static constexpr _Tp& Get(_Props& p) { return const_cast<_Tp&>(std::get<_Idx>(p.m_props)); }
  };

  template<auto _Idx, typename _Props>
  static constexpr auto _IndexSorted = std::tuple_element_t<_Idx, typename _Props::PropsSortedIndices>::value;

  template<std::size_t... _Idxs>
  static constexpr auto _GetIndexSorted(std::index_sequence<_Idxs...>, std::size_t i) {
    constexpr std::size_t arr[] = {_Idxs...};
    return arr[i];
  }

  template<template<auto, typename> typename _GetFunc,
           auto _Head, auto _Tail, typename _KeyTp, typename _Props, typename _Func>
  static bool _Search(_KeyTp k, _Props& p, _Func f) {
    using PropsTuple = typename _Props::PropsTuple;
    constexpr auto _SearchSz = _Tail - _Head;
    if constexpr (_SearchSz == 0) {
      return false;
    } else if constexpr (_SearchSz == 1) {
      constexpr auto _HeadIdx = _GetIndexSorted(typename _Props::PropsSortedIndices{}, _Head);
      constexpr _KeyTp _HeadK = std::tuple_element_t<_HeadIdx, PropsTuple>::FourCC::value;
      constexpr _KeyTp _HeadKS = SBIG(_HeadK);
      if (_HeadKS == k) {
        f(_GetFunc<_HeadIdx, _Props>::Get(p));
        return true;
      }
      return false;
    } else {
      constexpr auto _Mid = (_Head + (_Tail - 1)) / 2;
      constexpr auto _MidIdx = _GetIndexSorted(typename _Props::PropsSortedIndices{}, _Mid);
      constexpr _KeyTp _MidK = std::tuple_element_t<_MidIdx, PropsTuple>::FourCC::value;
      constexpr _KeyTp _MidKS = SBIG(_MidK);
      if (_MidKS == k) {
        f(_GetFunc<_MidIdx, _Props>::Get(p));
        return true;
      } else if (_MidKS < k) {
        return _Search<_GetFunc, _Mid + 1, _Tail>(k, p, f);
      } else {
        return _Search<_GetFunc, _Head, _Mid>(k, p, f);
      }
    }
  }

public:
  /* Static utilities */
  template<typename _PropsTuple>
  using Count = _Count<0, 0, _PropsTuple>;
  template<auto _Key, typename _Props>
  using GetElement = _Match<0, std::integral_constant<decltype(_Key), _Key>, typename _Props::PropsTuple>;
  template<typename _Type, typename _Props>
  using GetStoreIndex = _GetStoreIndex<0, 0, _Type, typename _Props::PropsTuple>;
  template<typename _Props>
  using Validate = _Validate<typename _Props::PropsTuple, typename _Props::PropsSortedIndices>;
  template<auto _Key, typename _Props>
  static constexpr decltype(auto) Get(_Props& p) {
    decltype(auto) prop = _PropGetFunc<GetElement<_Key, _Props>::Index, _Props>::Get(p);
    using Tp = std::decay_t<decltype(prop)>;
    if constexpr (Tp::IsBool)
      return p.m_boolProps[GetStoreIndex<Tp, _Props>::Index];
    else
      return prop;
  }

  /* Dynamic utilities */
  template<typename _KeyTp, typename _Props, typename _Func>
  static bool Search(_KeyTp k, _Props& p, _Func f) {
    return _Search<_PropGetFunc, 0, std::tuple_size_v<typename _Props::PropsTuple>>(hecl::SBig(k), p, f);
  }

  /* Stream utilities */
  template<typename _Props>
  static void Read(athena::io::IStreamReader& r, _Props& props) {
    constexpr FourCC RefType = uint32_t(_Props::Type);
    using FCCType = typename std::tuple_element_t<0, typename _Props::PropsTuple>::FourCC::value_type;
    DNAFourCC clsId;
    clsId.read(r);
    if (clsId != RefType) {
      LogModule.report(logvisor::Warning, fmt("non {} provided to {} parser"), RefType, RefType);
      return;
    }
    clsId.read(r);
    while (clsId != SBIG('_END')) {
      if (!Search(FCCType(clsId.toUint32()), props, [&](auto&& p) {
        using Tp = std::decay_t<decltype(p)>;
        if constexpr (Tp::IsBool) {
          r.readUint32Big();
          p.GetRef(props) = r.readBool();
        } else {
          p.GetRef(props).read(r);
        }
      })) {
        LogModule.report(logvisor::Fatal, fmt("Unknown {} class {} @{}"), RefType, clsId, r.position());
      }
      clsId.read(r);
    }
  }

  template<typename _Props>
  static void Write(athena::io::IStreamWriter& w, _Props& props) {
    constexpr DNAFourCC RefType = uint32_t(_Props::Type);
    RefType.write(w);
    std::apply([&](auto&&... p) {
      ([&](auto&& p) {
        if (p.ShouldStore(props)) {
          using Tp = std::decay_t<decltype(p)>;
          constexpr DNAFourCC ThisType = uint32_t(Tp::FourCC::value);
          ThisType.write(w);
          if constexpr (Tp::IsBool) {
            w.writeBytes("CNST", 4);
            w.writeBool(p.GetRef(props));
          } else {
            p.GetRef(props).write(w);
          }
        }
      }(p), ...);
    }, props.m_props);
    w.writeBytes("_END", 4);
  }

  template<typename _Props>
  static void BinarySize(std::size_t& s, _Props& props) {
    constexpr DNAFourCC RefType = uint32_t(_Props::Type);
    RefType.binarySize(s);
    std::apply([&](auto&&... p) {
      ([&](auto&& p) {
        if (p.ShouldStore(props)) {
          using Tp = std::decay_t<decltype(p)>;
          constexpr DNAFourCC ThisType = uint32_t(Tp::FourCC::value);
          ThisType.binarySize(s);
          if constexpr (Tp::IsBool) {
            s += 5;
          } else {
            p.GetRef(props).binarySize(s);
          }
        }
      }(p), ...);
    }, props.m_props);
    s += 4;
  }

  template<typename _Props>
  static void ReadYaml(athena::io::YAMLDocReader& r, _Props& props) {
    constexpr FourCC RefType = uint32_t(_Props::Type);
    using FCCType = typename std::tuple_element_t<0, typename _Props::PropsTuple>::FourCC::value_type;

    for (const auto& [key, value] : r.getCurNode()->m_mapChildren) {
      if (key == "DNAType"sv)
        continue;
      if (key.size() < 4) {
        LogModule.report(logvisor::Warning, fmt("short FourCC in element '{}'"), key);
        continue;
      }

      if (auto rec = r.enterSubRecord(key)) {
        const DNAFourCC clsId = key.c_str();
        if (!Search(FCCType(clsId.toUint32()), props, [&](auto&& p) {
          using Tp = std::decay_t<decltype(p)>;
          if constexpr (Tp::IsBool) {
            p.GetRef(props) = r.readBool();
          } else {
            p.GetRef(props).read(r);
          }
        })) {
          LogModule.report(logvisor::Fatal, fmt("Unknown {} class {}"), RefType, clsId);
        }
      }
    }
  }

  template<typename _Props>
  static void WriteYaml(athena::io::YAMLDocWriter& w, _Props& props) {
    std::apply([&](auto&&... p) {
      ([&](auto&& p) {
        if (p.ShouldStore(props)) {
          using Tp = std::decay_t<decltype(p)>;
          constexpr DNAFourCC ThisType = uint32_t(Tp::FourCC::value);
          if (auto rec = w.enterSubRecord(ThisType.toStringView())) {
            if constexpr (Tp::IsBool) {
              w.writeBool(p.GetRef(props));
            } else {
              p.GetRef(props).write(w);
            }
          }
        }
      }(p), ...);
    }, props.m_props);
  }

  template<typename _Props>
  static void GatherDependencies(std::vector<hecl::ProjectPath>& deps, _Props& props) {
    std::apply([&](auto&&... p) {
      ([&](auto&& p) {
        using Tp = std::decay_t<decltype(p)>;
        if constexpr (!Tp::IsBool)
          p.GetRef(props).gatherDependencies(deps);
      }(p), ...);
    }, props.m_props);
  }
};

template <auto FCC, typename T, bool _DefaultBool = false>
struct PP : T {
  static constexpr bool IsBool = false;
  static constexpr bool IsCP = false;
  static constexpr bool IsDP = false;
  using Type = T;
  using FourCC = std::integral_constant<decltype(FCC), FCC>;
  template<typename _Props>
  T& GetRef(_Props& p) { return *this; }
  template<typename _Props>
  bool ShouldStore(_Props& p) { return T::operator bool(); }
};

/* Boolean types are stored in a separate bitset */
template <auto FCC, bool _DefaultBool>
struct PP<FCC, bool, _DefaultBool> {
  static constexpr bool IsBool = true;
  static constexpr bool IsCP = false;
  static constexpr bool IsDP = false;
  using Type = bool;
  using FourCC = std::integral_constant<decltype(FCC), FCC>;
  using DefaultBool = std::integral_constant<bool, _DefaultBool>;
  template<typename _Props>
  decltype(auto) GetRef(_Props& p) {
    using Tp = std::decay_t<decltype(*this)>;
    return p.m_boolProps[PPUtils::GetStoreIndex<Tp, _Props>::Index];
  }
  template<typename _Props>
  bool ShouldStore(_Props& p) { return GetRef(p) != DefaultBool::value; }
};

template <typename _PropsTuple, auto... _SortedTypes>
struct _PPSet {
  using Self = _PPSet<_PropsTuple, _SortedTypes...>;
  using PropsTuple = _PropsTuple;
  using PropsSortedIndices = std::index_sequence<PPUtils::GetElement<_SortedTypes, Self>::Index...>;
  static_assert(std::tuple_size_v<PropsTuple> == PropsSortedIndices::size(),
                "Tuple and sorted indices size mismatch.");
  using PropsBitset = std::bitset<PPUtils::Count<PropsTuple>::BoolCount>;
private:
#if !defined(__GNUC__) || defined(__clang__)
  /* Disabled on GCC for its annoying slowness. */
  static constexpr PPUtils::Validate<Self> _validate{};
#endif
};

template <ParticleType _Type, typename _PropsTuple, auto... _SortedTypes>
struct PPSet : _PPSet<_PropsTuple, _SortedTypes...> {
  static constexpr ParticleType Type = _Type;
  using Base = _PPSet<_PropsTuple, _SortedTypes...>;
  typename Base::PropsTuple m_props;
  typename Base::PropsBitset m_boolProps;
  PPSet() {
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

template <class _Basis>
struct PPImpl : BigDNA {
  AT_DECL_EXPLICIT_DNA_YAML
  using EPP = typename _Basis::EPP;
  template<EPP _Key>
  decltype(auto) get() { return PPUtils::Get<_Key>(m_props); }

  void _read(athena::io::IStreamReader& r) { PPUtils::Read(r, m_props); }
  void _write(athena::io::IStreamWriter& w) { PPUtils::Write(w, m_props); }
  void _binarySize(std::size_t& s) { PPUtils::BinarySize(s, m_props); }
  void _read(athena::io::YAMLDocReader& r) { PPUtils::ReadYaml(r, m_props); }
  void _write(athena::io::YAMLDocWriter& w) { PPUtils::WriteYaml(w, m_props); }

  void gatherDependencies(std::vector<hecl::ProjectPath>& deps) const {
    PPUtils::GatherDependencies(deps, const_cast<typename _Basis::Properties&>(m_props));
  }

protected:
  typename _Basis::Properties m_props;
};

class PEUtils {
  template<auto _Idx, typename _Elems>
  struct _ElemGetFunc {
    using _Tp = std::tuple_element_t<_Idx, typename _Elems::PropsTuple>;
    static constexpr _Tp Get(_Elems& e) { return _Tp(); }
  };

public:
  /* Dynamic utilities */
  template<typename _KeyTp, typename _Elems, typename _Func>
  static bool Search(_KeyTp k, _Elems& e, _Func f) {
    return PPUtils::_Search<_ElemGetFunc, 0, std::tuple_size_v<typename _Elems::PropsTuple>>(hecl::SBig(k), e, f);
  }

  /* Stream utilities */
  template<typename _Elems>
  static void Read(athena::io::IStreamReader& r, _Elems& elems) {
    using FCCType = typename std::tuple_element_t<0, typename _Elems::PropsTuple>::FourCC::value_type;
    DNAFourCC clsId;
    clsId.read(r);
    if (clsId == FOURCC('NONE')) {
      elems.m_elem.reset();
      return;
    }
    if (!Search(FCCType(clsId.toUint32()), elems, [&](auto&& p) {
      using Tp = std::decay_t<decltype(p)>;
      elems.m_elem = std::make_unique<typename Tp::Type>();
      elems.m_elem->read(r);
    })) {
      LogModule.report(logvisor::Fatal, fmt("Unknown {} class {} @{}"), _Elems::PtrType::TypeName, clsId, r.position());
    }
  }

  template<typename _Elems>
  static void Write(athena::io::IStreamWriter& w, _Elems& elems) {
    if (elems.m_elem) {
      w.writeBytes(elems.m_elem->ClassID().data(), 4);
      elems.m_elem->write(w);
    } else {
      w.writeBytes("NONE", 4);
    }
  }

  template<typename _Elems>
  static void BinarySize(std::size_t& s, _Elems& elems) {
    if (elems.m_elem)
      elems.m_elem->binarySize(s);
    s += 4;
  }

  template<typename _Elems>
  static void ReadYaml(athena::io::YAMLDocReader& r, _Elems& elems) {
    using FCCType = typename std::tuple_element_t<0, typename _Elems::PropsTuple>::FourCC::value_type;

    const auto& mapChildren = r.getCurNode()->m_mapChildren;
    if (mapChildren.empty()) {
      elems.m_elem.reset();
      return;
    }

    const auto& [key, value] = mapChildren[0];
    if (key.size() < 4)
      LogModule.report(logvisor::Fatal, fmt("short FourCC in element '{}'"), key);

    if (auto rec = r.enterSubRecord(key)) {
      const DNAFourCC clsId = key.c_str();
      if (!Search(FCCType(clsId.toUint32()), elems, [&](auto&& p) {
        using Tp = std::decay_t<decltype(p)>;
        elems.m_elem = std::make_unique<typename Tp::Type>();
        elems.m_elem->read(r);
      })) {
        LogModule.report(logvisor::Fatal, fmt("Unknown {} class {}"), _Elems::PtrType::TypeName, clsId);
      }
    }
  }

  template<typename _Elems>
  static void WriteYaml(athena::io::YAMLDocWriter& w, _Elems& elems) {
    if (elems.m_elem)
      if (auto rec = w.enterSubRecord(elems.m_elem->ClassID()))
        elems.m_elem->write(w);
  }
};

template <auto FCC, typename T>
struct PE {
  static constexpr bool IsBool = false;
  using Type = T;
  using FourCC = std::integral_constant<decltype(FCC), FCC>;
};

template <typename _PtrType, typename _PropsTuple, auto... _SortedTypes>
struct PESet : _PPSet<_PropsTuple, _SortedTypes...> {
  using PtrType = _PtrType;
  std::unique_ptr<_PtrType> m_elem;
};

template <class _Basis>
struct PEImpl : BigDNA {
  AT_DECL_EXPLICIT_DNA_YAML

  void _read(athena::io::IStreamReader& r) { PEUtils::Read(r, m_elems); }
  void _write(athena::io::IStreamWriter& w) { PEUtils::Write(w, m_elems); }
  void _binarySize(std::size_t& s) { PEUtils::BinarySize(s, m_elems); }
  void _read(athena::io::YAMLDocReader& r) { PEUtils::ReadYaml(r, m_elems); }
  void _write(athena::io::YAMLDocWriter& w) { PEUtils::WriteYaml(w, m_elems); }

  void gatherDependencies(std::vector<hecl::ProjectPath>& deps) const {
    _Basis::gatherDependencies(deps, m_elems.m_elem);
  }

  operator bool() const { return m_elems.m_elem.operator bool(); }
  auto* get() const { return m_elems.m_elem.get(); }
  auto* operator->() const { return get(); }
  void reset() { m_elems.m_elem.reset(); }
private:
  typename _Basis::Elements m_elems;
};

struct IElement : BigDNAVYaml {
  Delete _d;
  ~IElement() override = default;
  virtual std::string_view ClassID() const = 0;
  std::string_view DNATypeV() const override { return ClassID(); }
};

struct IRealElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "RealElement"sv;
};
struct _RealElementFactory {
  enum EPP : uint32_t {
    LFTW = SBIG('LFTW'),
    CNST = SBIG('CNST'),
    CHAN = SBIG('CHAN'),
    ADD_ = SBIG('ADD_'),
    CLMP = SBIG('CLMP'),
    KEYE = SBIG('KEYE'),
    KEYP = SBIG('KEYP'),
    IRND = SBIG('IRND'),
    RAND = SBIG('RAND'),
    MULT = SBIG('MULT'),
    PULS = SBIG('PULS'),
    SCAL = SBIG('SCAL'),
    RLPT = SBIG('RLPT'),
    SINE = SBIG('SINE'),
    ISWT = SBIG('ISWT'),
    CLTN = SBIG('CLTN'),
    CEQL = SBIG('CEQL'),
    PAP1 = SBIG('PAP1'),
    PAP2 = SBIG('PAP2'),
    PAP3 = SBIG('PAP3'),
    PAP4 = SBIG('PAP4'),
    PAP5 = SBIG('PAP5'),
    PAP6 = SBIG('PAP6'),
    PAP7 = SBIG('PAP7'),
    PAP8 = SBIG('PAP8'),
    PSLL = SBIG('PSLL'),
    PRLW = SBIG('PRLW'),
    SUB_ = SBIG('SUB_'),
    VMAG = SBIG('VMAG'),
    VXTR = SBIG('VXTR'),
    VYTR = SBIG('VYTR'),
    VZTR = SBIG('VZTR'),
    CEXT = SBIG('CEXT'),
    ITRL = SBIG('ITRL'),
  };
  using Elements = PESet<
      IRealElement, std::tuple<
          PE<LFTW, struct RELifetimeTween>,
          PE<CNST, struct REConstant>,
          PE<CHAN, struct RETimeChain>,
          PE<ADD_, struct REAdd>,
          PE<CLMP, struct REClamp>,
          PE<KEYE, struct REKeyframeEmitter>,
          PE<KEYP, struct REKeyframeEmitter>,
          PE<IRND, struct REInitialRandom>,
          PE<RAND, struct RERandom>,
          PE<MULT, struct REMultiply>,
          PE<PULS, struct REPulse>,
          PE<SCAL, struct RETimeScale>,
          PE<RLPT, struct RELifetimePercent>,
          PE<SINE, struct RESineWave>,
          PE<ISWT, struct REInitialSwitch>,
          PE<CLTN, struct RECompareLessThan>,
          PE<CEQL, struct RECompareEquals>,
          PE<PAP1, struct REParticleAdvanceParam1>,
          PE<PAP2, struct REParticleAdvanceParam2>,
          PE<PAP3, struct REParticleAdvanceParam3>,
          PE<PAP4, struct REParticleAdvanceParam4>,
          PE<PAP5, struct REParticleAdvanceParam5>,
          PE<PAP6, struct REParticleAdvanceParam6>,
          PE<PAP7, struct REParticleAdvanceParam7>,
          PE<PAP8, struct REParticleAdvanceParam8>,
          PE<PSLL, struct REParticleSizeOrLineLength>,
          PE<PRLW, struct REParticleRotationOrLineWidth>,
          PE<SUB_, struct RESubtract>,
          PE<VMAG, struct REVectorMagnitude>,
          PE<VXTR, struct REVectorXToReal>,
          PE<VYTR, struct REVectorYToReal>,
          PE<VZTR, struct REVectorZToReal>,
          PE<CEXT, struct RECEXT>,
          PE<ITRL, struct REIntTimesReal>>,
      ADD_, CEQL, CEXT, CHAN, CLMP, CLTN, CNST, IRND, ISWT, ITRL, KEYE, KEYP, LFTW, MULT, PAP1, PAP2, PAP3, PAP4, PAP5,
      PAP6, PAP7, PAP8, PRLW, PSLL, PULS, RAND, RLPT, SCAL, SINE, SUB_, VMAG, VXTR, VYTR, VZTR>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IRealElement>& elemPtr) {}
};
using RealElementFactory = PEImpl<_RealElementFactory>;

struct IIntElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "IntElement"sv;
};
struct _IntElementFactory {
  enum EPP : uint32_t {
    KEYE = SBIG('KEYE'),
    KEYP = SBIG('KEYP'),
    DETH = SBIG('DETH'),
    CLMP = SBIG('CLMP'),
    CHAN = SBIG('CHAN'),
    ADD_ = SBIG('ADD_'),
    CNST = SBIG('CNST'),
    IMPL = SBIG('IMPL'),
    ILPT = SBIG('ILPT'),
    IRND = SBIG('IRND'),
    PULS = SBIG('PULS'),
    MULT = SBIG('MULT'),
    SPAH = SBIG('SPAH'),
    RAND = SBIG('RAND'),
    TSCL = SBIG('TSCL'),
    GTCP = SBIG('GTCP'),
    MODU = SBIG('MODU'),
    SUB_ = SBIG('SUB_'),
  };
  using Elements = PESet<
      IIntElement, std::tuple<
          PE<KEYE, struct IEKeyframeEmitter>,
          PE<KEYP, struct IEKeyframeEmitter>,
          PE<DETH, struct IEDeath>,
          PE<CLMP, struct IEClamp>,
          PE<CHAN, struct IETimeChain>,
          PE<ADD_, struct IEAdd>,
          PE<CNST, struct IEConstant>,
          PE<IMPL, struct IEImpulse>,
          PE<ILPT, struct IELifetimePercent>,
          PE<IRND, struct IEInitialRandom>,
          PE<PULS, struct IEPulse>,
          PE<MULT, struct IEMultiply>,
          PE<SPAH, struct IESampleAndHold>,
          PE<RAND, struct IERandom>,
          PE<TSCL, struct IETimeScale>,
          PE<GTCP, struct IEGTCP>,
          PE<MODU, struct IEModulo>,
          PE<SUB_, struct IESubtract>>,
      ADD_, CHAN, CLMP, CNST, DETH, GTCP, ILPT, IMPL, IRND, KEYE, KEYP, MODU, MULT, PULS, RAND, SPAH, SUB_, TSCL>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IIntElement>& elemPtr) {}
};
using IntElementFactory = PEImpl<_IntElementFactory>;

struct IVectorElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "VectorElement"sv;
};
struct _VectorElementFactory {
  enum EPP : uint32_t {
    CONE = SBIG('CONE'),
    CHAN = SBIG('CHAN'),
    ANGC = SBIG('ANGC'),
    ADD_ = SBIG('ADD_'),
    CCLU = SBIG('CCLU'),
    CNST = SBIG('CNST'),
    CIRC = SBIG('CIRC'),
    KEYE = SBIG('KEYE'),
    KEYP = SBIG('KEYP'),
    MULT = SBIG('MULT'),
    RTOV = SBIG('RTOV'),
    PULS = SBIG('PULS'),
    PVEL = SBIG('PVEL'),
    SPOS = SBIG('SPOS'),
    PLCO = SBIG('PLCO'),
    PLOC = SBIG('PLOC'),
    PSOR = SBIG('PSOR'),
    PSOF = SBIG('PSOF'),
  };
  using Elements = PESet<
      IVectorElement, std::tuple<
          PE<CONE, struct VECone>,
          PE<CHAN, struct VETimeChain>,
          PE<ANGC, struct VEAngleCone>,
          PE<ADD_, struct VEAdd>,
          PE<CCLU, struct VECircleCluster>,
          PE<CNST, struct VEConstant>,
          PE<CIRC, struct VECircle>,
          PE<KEYE, struct VEKeyframeEmitter>,
          PE<KEYP, struct VEKeyframeEmitter>,
          PE<MULT, struct VEMultiply>,
          PE<RTOV, struct VERealToVector>,
          PE<PULS, struct VEPulse>,
          PE<PVEL, struct VEParticleVelocity>,
          PE<SPOS, struct VESPOS>,
          PE<PLCO, struct VEPLCO>,
          PE<PLOC, struct VEPLOC>,
          PE<PSOR, struct VEPSOR>,
          PE<PSOF, struct VEPSOF>>,
      ADD_, ANGC, CCLU, CHAN, CIRC, CNST, CONE, KEYE, KEYP, MULT, PLCO, PLOC, PSOF, PSOR, PULS, PVEL, RTOV, SPOS>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IVectorElement>& elemPtr) {}
};
using VectorElementFactory = PEImpl<_VectorElementFactory>;

struct IColorElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "ColorElement"sv;
};
struct _ColorElementFactory {
  enum EPP : uint32_t {
    KEYE = SBIG('KEYE'),
    KEYP = SBIG('KEYP'),
    CNST = SBIG('CNST'),
    CHAN = SBIG('CHAN'),
    CFDE = SBIG('CFDE'),
    FADE = SBIG('FADE'),
    PULS = SBIG('PULS'),
  };
  using Elements = PESet<
      IColorElement, std::tuple<
          PE<KEYE, struct CEKeyframeEmitter>,
          PE<KEYP, struct CEKeyframeEmitter>,
          PE<CNST, struct CEConstant>,
          PE<CHAN, struct CETimeChain>,
          PE<CFDE, struct CEFadeEnd>,
          PE<FADE, struct CEFade>,
          PE<PULS, struct CEPulse>>,
      CFDE, CHAN, CNST, FADE, KEYE, KEYP, PULS>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IColorElement>& elemPtr) {}
};
using ColorElementFactory = PEImpl<_ColorElementFactory>;

struct IModVectorElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "ModVectorElement"sv;
};
struct _ModVectorElementFactory {
  enum EPP : uint32_t {
    IMPL = SBIG('IMPL'),
    EMPL = SBIG('EMPL'),
    CHAN = SBIG('CHAN'),
    BNCE = SBIG('BNCE'),
    CNST = SBIG('CNST'),
    GRAV = SBIG('GRAV'),
    EXPL = SBIG('EXPL'),
    SPOS = SBIG('SPOS'),
    LMPL = SBIG('LMPL'),
    PULS = SBIG('PULS'),
    WIND = SBIG('WIND'),
    SWRL = SBIG('SWRL'),
  };
  using Elements = PESet<
      IModVectorElement, std::tuple<
          PE<IMPL, struct MVEImplosion>,
          PE<EMPL, struct MVEExponentialImplosion>,
          PE<CHAN, struct MVETimeChain>,
          PE<BNCE, struct MVEBounce>,
          PE<CNST, struct MVEConstant>,
          PE<GRAV, struct MVEGravity>,
          PE<EXPL, struct MVEExplode>,
          PE<SPOS, struct MVESetPosition>,
          PE<LMPL, struct MVELinearImplosion>,
          PE<PULS, struct MVEPulse>,
          PE<WIND, struct MVEWind>,
          PE<SWRL, struct MVESwirl>>,
      BNCE, CHAN, CNST, EMPL, EXPL, GRAV, IMPL, LMPL, PULS, SPOS, SWRL, WIND>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IModVectorElement>& elemPtr) {}
};
using ModVectorElementFactory = PEImpl<_ModVectorElementFactory>;

struct IEmitterElement : IElement {
  Delete _d2;
  static constexpr std::string_view TypeName = "EmitterElement"sv;
};
struct _EmitterElementFactory {
  enum EPP : uint32_t {
    SETR = SBIG('SETR'),
    SEMR = SBIG('SEMR'),
    SPHE = SBIG('SPHE'),
    ASPH = SBIG('ASPH'),
  };
  using Elements = PESet<
      IEmitterElement, std::tuple<
          PE<SETR, struct EESimpleEmitterTR>,
          PE<SEMR, struct EESimpleEmitter>,
          PE<SPHE, struct VESphere>,
          PE<ASPH, struct VEAngleSphere>>,
      ASPH, SEMR, SETR, SPHE>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IEmitterElement>& elemPtr) {}
};
using EmitterElementFactory = PEImpl<_EmitterElementFactory>;

struct IUVElement : IElement {
  Delete _d2;
  virtual void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) const = 0;
  static constexpr std::string_view TypeName = "UVElement"sv;
};

struct BoolHelper : IElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  bool value = false;
  operator bool() const { return value; }
  BoolHelper& operator=(bool val) {
    value = val;
    return *this;
  }
  std::string_view ClassID() const override { return "BoolHelper"sv; }
};

template <typename Tp>
struct ValueHelper : BigDNA {
  AT_DECL_EXPLICIT_DNA_YAML

  void _read(athena::io::IStreamReader& r) {
    hecl::DNAFourCC ValueType;
    ValueType.read(r);
    if (ValueType == FOURCC('CNST'))
      athena::io::Read<athena::io::PropType::None>::Do<Tp, athena::Endian::Big>({}, value.emplace(), r);
    else
      value = std::nullopt;
  }
  void _write(athena::io::IStreamWriter& w) {
    if (value) {
      w.writeBytes("CNST", 4);
      athena::io::Write<athena::io::PropType::None>::Do<Tp, athena::Endian::Big>({}, *value, w);
    } else {
      w.writeBytes("NONE", 4);
    }
  }
  void _binarySize(std::size_t& s) {
    s += 4;
    if (value)
      athena::io::BinarySize<athena::io::PropType::None>::Do<Tp, athena::Endian::Big>({}, *value, s);
  }
  void _read(athena::io::YAMLDocReader& r) {
    athena::io::ReadYaml<athena::io::PropType::None>::Do<Tp, athena::Endian::Big>({}, value.emplace(), r);
  }
  void _write(athena::io::YAMLDocWriter& w) {
    athena::io::WriteYaml<athena::io::PropType::None>::Do<Tp, athena::Endian::Big>({}, *value, w);
  }

  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) {}

  std::optional<Tp> value = {};
  void emplace(Tp val) { value.emplace(val); }
  Tp operator*() const { return *value; }
  operator bool() const { return value.operator bool(); }
};

struct RELifetimeTween : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "LFTW"sv; }
};

struct REConstant : IRealElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  Value<float> val;
  std::string_view ClassID() const override { return "CNST"sv; }
};

struct RETimeChain : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "CHAN"sv; }
};

struct REAdd : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "ADD_"sv; }
};

struct REClamp : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory min;
  RealElementFactory max;
  RealElementFactory val;
  std::string_view ClassID() const override { return "CLMP"sv; }
};

struct REKeyframeEmitter : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  Value<atUint32> percentageTween;
  Value<atUint32> unk1;
  Value<bool> loop;
  Value<atUint8> unk2;
  Value<atUint32> loopEnd;
  Value<atUint32> loopStart;
  Value<atUint32> count;
  Vector<float, AT_DNA_COUNT(count)> keys;
  std::string_view ClassID() const override { return percentageTween ? "KEYP"sv : "KEYE"sv; }
};

struct REInitialRandom : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "IRND"sv; }
};

struct RERandom : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "RAND"sv; }
};

struct REMultiply : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "MULT"sv; }
};

struct REPulse : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory aDuration;
  IntElementFactory bDuration;
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "PULS"sv; }
};

struct RETimeScale : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory dv;
  std::string_view ClassID() const override { return "SCAL"sv; }
};

struct RELifetimePercent : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory percent;
  std::string_view ClassID() const override { return "RLPT"sv; }
};

struct RESineWave : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory frequency;
  RealElementFactory amplitude;
  RealElementFactory phase;
  std::string_view ClassID() const override { return "SINE"sv; }
};

struct REInitialSwitch : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "ISWT"sv; }
};

struct RECompareLessThan : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory ca;
  RealElementFactory cb;
  RealElementFactory pass;
  RealElementFactory fail;
  std::string_view ClassID() const override { return "CLTN"sv; }
};

struct RECompareEquals : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory ca;
  RealElementFactory cb;
  RealElementFactory pass;
  RealElementFactory fail;
  std::string_view ClassID() const override { return "CEQL"sv; }
};

struct REParticleAdvanceParam1 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP1"sv; }
};

struct REParticleAdvanceParam2 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP2"sv; }
};

struct REParticleAdvanceParam3 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP3"sv; }
};

struct REParticleAdvanceParam4 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP4"sv; }
};

struct REParticleAdvanceParam5 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP5"sv; }
};

struct REParticleAdvanceParam6 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP6"sv; }
};

struct REParticleAdvanceParam7 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP7"sv; }
};

struct REParticleAdvanceParam8 : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PAP8"sv; }
};

struct REParticleSizeOrLineLength : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PSLL"sv; }
};

struct REParticleRotationOrLineWidth : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PRLW"sv; }
};

struct RESubtract : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "SUB_"sv; }
};

struct REVectorMagnitude : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory vec;
  std::string_view ClassID() const override { return "VMAG"sv; }
};

struct REVectorXToReal : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory vec;
  std::string_view ClassID() const override { return "VXTR"sv; }
};

struct REVectorYToReal : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory vec;
  std::string_view ClassID() const override { return "VYTR"sv; }
};

struct REVectorZToReal : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory vec;
  std::string_view ClassID() const override { return "VZTR"sv; }
};

struct RECEXT : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory index;
  std::string_view ClassID() const override { return "CEXT"sv; }
};

struct REIntTimesReal : IRealElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "ITRL"sv; }
};

struct IEKeyframeEmitter : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  Value<atUint32> percentageTween;
  Value<atUint32> unk1;
  Value<bool> loop;
  Value<atUint8> unk2;
  Value<atUint32> loopEnd;
  Value<atUint32> loopStart;
  Value<atUint32> count;
  Vector<atUint32, AT_DNA_COUNT(count)> keys;
  std::string_view ClassID() const override { return percentageTween ? "KEYP"sv : "KEYE"sv; }
};

struct IEDeath : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory passthrough;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "DETH"sv; }
};

struct IEClamp : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory min;
  IntElementFactory max;
  IntElementFactory val;
  std::string_view ClassID() const override { return "CLMP"sv; }
};

struct IETimeChain : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "CHAN"sv; }
};

struct IEAdd : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "ADD_"sv; }
};

struct IEConstant : IIntElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  Value<atUint32> val;
  std::string_view ClassID() const override { return "CNST"sv; }
};

struct IEImpulse : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory val;
  std::string_view ClassID() const override { return "IMPL"sv; }
};

struct IELifetimePercent : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory percent;
  std::string_view ClassID() const override { return "ILPT"sv; }
};

struct IEInitialRandom : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "IRND"sv; }
};

struct IEPulse : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory aDuration;
  IntElementFactory bDuration;
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "PULS"sv; }
};

struct IEMultiply : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "MULT"sv; }
};

struct IESampleAndHold : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory val;
  IntElementFactory waitMin;
  IntElementFactory waitMax;
  std::string_view ClassID() const override { return "SPAH"sv; }
};

struct IERandom : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "RAND"sv; }
};

struct IETimeScale : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory dv;
  std::string_view ClassID() const override { return "TSCL"sv; }
};

struct IEGTCP : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "GTCP"sv; }
};

struct IEModulo : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory a;
  IntElementFactory b;
  std::string_view ClassID() const override { return "MODU"sv; }
};

struct IESubtract : IIntElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory direction;
  IntElementFactory baseRadius;
  std::string_view ClassID() const override { return "SUB_"sv; }
};

struct VECone : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory a;
  RealElementFactory b;
  std::string_view ClassID() const override { return "CONE"sv; }
};

struct VETimeChain : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory a;
  VectorElementFactory b;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "CHAN"sv; }
};

struct VEAngleCone : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory angleXBias;
  RealElementFactory angleYBias;
  RealElementFactory angleXRange;
  RealElementFactory angleYRange;
  RealElementFactory magnitude;
  std::string_view ClassID() const override { return "ANGC"sv; }
};

struct VEAdd : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory a;
  VectorElementFactory b;
  std::string_view ClassID() const override { return "ADD_"sv; }
};

struct VECircleCluster : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory circleOffset;
  VectorElementFactory circleNormal;
  IntElementFactory cycleFrames;
  RealElementFactory randomFactor;
  std::string_view ClassID() const override { return "CCLU"sv; }
};

struct VEConstant : IVectorElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  RealElementFactory comps[3];
  std::string_view ClassID() const override { return "CNST"sv; }
};

struct VECircle : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory circleOffset;
  VectorElementFactory circleNormal;
  RealElementFactory angleConstant;
  RealElementFactory angleLinear;
  RealElementFactory circleRadius;
  std::string_view ClassID() const override { return "CIRC"sv; }
};

struct VEKeyframeEmitter : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  Value<atUint32> percentageTween;
  Value<atUint32> unk1;
  Value<bool> loop;
  Value<atUint8> unk2;
  Value<atUint32> loopEnd;
  Value<atUint32> loopStart;
  Value<atUint32> count;
  Vector<atVec3f, AT_DNA_COUNT(count)> keys;
  std::string_view ClassID() const override { return percentageTween ? "KEYP"sv : "KEYE"sv; }
};

struct VEMultiply : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory a;
  VectorElementFactory b;
  std::string_view ClassID() const override { return "MULT"sv; }
};

struct VERealToVector : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory a;
  std::string_view ClassID() const override { return "RTOV"sv; }
};

struct VEPulse : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory aDuration;
  IntElementFactory bDuration;
  VectorElementFactory a;
  VectorElementFactory b;
  std::string_view ClassID() const override { return "PULS"sv; }
};

struct VEParticleVelocity : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PVEL"sv; }
};

struct VESPOS : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory a;
  std::string_view ClassID() const override { return "SPOS"sv; }
};

struct VEPLCO : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PLCO"sv; }
};

struct VEPLOC : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PLOC"sv; }
};

struct VEPSOR : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PSOR"sv; }
};

struct VEPSOF : IVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "PSOF"sv; }
};

struct CEKeyframeEmitter : IColorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  Value<atUint32> percentageTween;
  Value<atUint32> unk1;
  Value<bool> loop;
  Value<atUint8> unk2;
  Value<atUint32> loopEnd;
  Value<atUint32> loopStart;
  Value<atUint32> count;
  Vector<atVec4f, AT_DNA_COUNT(count)> keys;
  std::string_view ClassID() const override { return percentageTween ? "KEYP"sv : "KEYE"sv; }
};

struct CEConstant : IColorElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  RealElementFactory comps[4];
  std::string_view ClassID() const override { return "CNST"sv; }
};

struct CETimeChain : IColorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  ColorElementFactory a;
  ColorElementFactory b;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "CHAN"sv; }
};

struct CEFadeEnd : IColorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  ColorElementFactory a;
  ColorElementFactory b;
  RealElementFactory startFrame;
  RealElementFactory endFrame;
  std::string_view ClassID() const override { return "CFDE"sv; }
};

struct CEFade : IColorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  ColorElementFactory a;
  ColorElementFactory b;
  RealElementFactory endFrame;
  std::string_view ClassID() const override { return "FADE"sv; }
};

struct CEPulse : IColorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory aDuration;
  IntElementFactory bDuration;
  ColorElementFactory a;
  ColorElementFactory b;
  std::string_view ClassID() const override { return "PULS"sv; }
};

struct MVEImplosion : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory implodePoint;
  RealElementFactory velocityScale;
  RealElementFactory maxRadius;
  RealElementFactory minRadius;
  BoolHelper enableMinRadius;
  std::string_view ClassID() const override { return "IMPL"sv; }
};

struct MVEExponentialImplosion : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory implodePoint;
  RealElementFactory velocityScale;
  RealElementFactory maxRadius;
  RealElementFactory minRadius;
  BoolHelper enableMinRadius;
  std::string_view ClassID() const override { return "EMPL"sv; }
};

struct MVETimeChain : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  ModVectorElementFactory a;
  ModVectorElementFactory b;
  IntElementFactory thresholdFrame;
  std::string_view ClassID() const override { return "CHAN"sv; }
};

struct MVEBounce : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory planePoint;
  VectorElementFactory planeNormal;
  RealElementFactory friction;
  RealElementFactory restitution;
  BoolHelper dieOnPenetrate;
  std::string_view ClassID() const override { return "BNCE"sv; }
};

struct MVEConstant : IModVectorElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  RealElementFactory comps[3];
  std::string_view ClassID() const override { return "CNST"sv; }
};

struct MVEGravity : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory acceleration;
  std::string_view ClassID() const override { return "GRAV"sv; }
};

struct MVEExplode : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  RealElementFactory impulseMagnitude;
  RealElementFactory falloffFactor;
  std::string_view ClassID() const override { return "EXPL"sv; }
};

struct MVESetPosition : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory position;
  std::string_view ClassID() const override { return "SPOS"sv; }
};

struct MVELinearImplosion : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory implodePoint;
  RealElementFactory velocityScale;
  RealElementFactory maxRadius;
  RealElementFactory minRadius;
  BoolHelper enableMinRadius;
  std::string_view ClassID() const override { return "LMPL"sv; }
};

struct MVEPulse : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  IntElementFactory aDuration;
  IntElementFactory bDuration;
  ModVectorElementFactory a;
  ModVectorElementFactory b;
  std::string_view ClassID() const override { return "PULS"sv; }
};

struct MVEWind : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory windVelocity;
  RealElementFactory factor;
  std::string_view ClassID() const override { return "WIND"sv; }
};

struct MVESwirl : IModVectorElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory helixPoint;
  VectorElementFactory curveBinormal;
  RealElementFactory filterGain;
  RealElementFactory tangentialVelocity;
  std::string_view ClassID() const override { return "SWRL"sv; }
};

struct EESimpleEmitter : IEmitterElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory position;
  VectorElementFactory velocity;
  std::string_view ClassID() const override { return "SEMR"sv; }
};

struct VESphere : IEmitterElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory sphereOrigin;
  RealElementFactory sphereRadius;
  RealElementFactory magnitude;
  std::string_view ClassID() const override { return "SPHE"sv; }
};

struct VEAngleSphere : IEmitterElement {
  AT_DECL_DNA_YAMLV_NO_TYPE
  VectorElementFactory sphereOrigin;
  RealElementFactory sphereRadius;
  RealElementFactory magnitude;
  RealElementFactory angleXBias;
  RealElementFactory angleYBias;
  RealElementFactory angleXRange;
  RealElementFactory angleYRange;
  std::string_view ClassID() const override { return "ASPH"sv; }
};

struct EESimpleEmitterTR : EESimpleEmitter {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  std::string_view ClassID() const override { return "SETR"sv; }
};

template <class IDType>
struct UVEConstant : IUVElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  AT_SUBDECL_DNA
  CastIDToZero<IDType> tex;
  std::string_view ClassID() const override { return "CNST"sv; }

  void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) const override {
    if (tex.isValid())
      g_curSpec->flattenDependencies(tex, pathsOut);
  }
};

template <class IDType>
struct UVEAnimTexture : IUVElement {
  AT_DECL_EXPLICIT_DNA_YAMLV_NO_TYPE
  AT_SUBDECL_DNA
  CastIDToZero<IDType> tex;
  IntElementFactory tileW;
  IntElementFactory tileH;
  IntElementFactory strideW;
  IntElementFactory strideH;
  IntElementFactory cycleFrames;
  Value<bool> loop = false;
  std::string_view ClassID() const override { return "ATEX"sv; }

  void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) const override {
    if (tex.isValid())
      g_curSpec->flattenDependencies(tex, pathsOut);
  }
};

template <class IDType>
struct _UVElementFactory {
  enum EPP : uint32_t {
    CNST = SBIG('CNST'),
    ATEX = SBIG('ATEX'),
  };
  using Elements = PESet<
      IUVElement, std::tuple<
          PE<CNST, struct UVEConstant<IDType>>,
          PE<ATEX, struct UVEAnimTexture<IDType>>>,
      ATEX, CNST>;
  static constexpr void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut,
                                           const std::unique_ptr<IUVElement>& elemPtr) {
    if (elemPtr)
      elemPtr->gatherDependencies(pathsOut);
  }
};
template <class IDType>
using UVElementFactory = PEImpl<_UVElementFactory<IDType>>;

template <class IDType>
struct SpawnSystemKeyframeData : BigDNA {
  Value<atUint32> a;
  Value<atUint32> b;
  Value<atUint32> endFrame;
  Value<atUint32> d;

  struct SpawnSystemKeyframeInfo : BigDNA {
    IDType id;
    Value<atUint32> a;
    Value<atUint32> b;
    Value<atUint32> c;
    AT_DECL_EXPLICIT_DNA_YAML
  };

  std::vector<std::pair<atUint32, std::vector<SpawnSystemKeyframeInfo>>> spawns;

  AT_DECL_EXPLICIT_DNA_YAML
  AT_SUBDECL_DNA

  operator bool() const { return spawns.size() != 0; }

  void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) const {
    for (const auto& p : spawns)
      for (const SpawnSystemKeyframeInfo& info : p.second)
        g_curSpec->flattenDependencies(info.id, pathsOut);
  }
};

template <class IDType>
struct ChildResourceFactory : BigDNA {
  IDType id;
  AT_DECL_EXPLICIT_DNA_YAML
  AT_SUBDECL_DNA
  operator bool() const { return id.isValid(); }
  void gatherDependencies(std::vector<hecl::ProjectPath>& pathsOut) const {
    if (id.isValid())
      g_curSpec->flattenDependencies(id, pathsOut);
  }
};

} // namespace DataSpec::DNAParticle
