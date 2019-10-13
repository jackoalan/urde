#pragma once

#include <optional>
#include <vector>

#include "DataSpec/DNACommon/CRSC.hpp"

#include "Runtime/CFactoryMgr.hpp"
#include "Runtime/CToken.hpp"
#include "Runtime/IOStreams.hpp"
#include "Runtime/IObj.hpp"
#include "Runtime/RetroTypes.hpp"
#include "Runtime/Collision/CMaterialList.hpp"

namespace urde {
class CDecalDescription;
class CGenDescription;
class CSimplePool;

using EWeaponCollisionResponseTypes = DataSpec::DNAParticle::EWeaponCollisionResponseTypes;

class CCollisionResponseData {
  static const EWeaponCollisionResponseTypes skWorldMaterialTable[32];
  static const s32 kInvalidSFX;
  std::vector<std::optional<TLockedToken<CGenDescription>>> x0_generators;
  std::vector<s32> x10_sfx;
  std::vector<std::optional<TLockedToken<CDecalDescription>>> x20_decals;
  float x30_RNGE;
  float x34_FOFF;

  void AddParticleSystemToResponse(EWeaponCollisionResponseTypes type, CInputStream& in, CSimplePool* resPool);
  bool CheckAndAddDecalToResponse(FourCC clsId, CInputStream& in, CSimplePool* resPool);
  bool CheckAndAddSoundFXToResponse(FourCC clsId, CInputStream& in);
  bool CheckAndAddParticleSystemToResponse(FourCC clsId, CInputStream& in, CSimplePool* resPool);
  bool CheckAndAddResourceToResponse(FourCC clsId, CInputStream& in, CSimplePool* resPool);

public:
  CCollisionResponseData(CInputStream& in, CSimplePool* resPool);
  const std::optional<TLockedToken<CGenDescription>>&
      GetParticleDescription(EWeaponCollisionResponseTypes) const;
  const std::optional<TLockedToken<CDecalDescription>>&
  GetDecalDescription(EWeaponCollisionResponseTypes type) const;
  s32 GetSoundEffectId(EWeaponCollisionResponseTypes) const;
  static EWeaponCollisionResponseTypes GetWorldCollisionResponseType(s32);
  static bool ResponseTypeIsEnemyShielded(EWeaponCollisionResponseTypes);
  static bool ResponseTypeIsEnemyNormal(EWeaponCollisionResponseTypes);
  static bool ResponseTypeIsEnemySpecial(EWeaponCollisionResponseTypes);
  float GetAudibleRange() const { return x30_RNGE; }
  float GetAudibleFallOff() const { return x34_FOFF; }
  static FourCC UncookedResType();
};

CFactoryFnReturn FCollisionResponseDataFactory(const SObjectTag& tag, CInputStream& in, const CVParamTransfer& vparms,
                                               CObjectReference*);
} // namespace urde
