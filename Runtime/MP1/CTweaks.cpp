#include "logvisor/logvisor.hpp"
#include "CTweaks.hpp"
#include "CResFactory.hpp"
#include "CResLoader.hpp"
#include "GameGlobalObjects.hpp"
#include "Editor/ProjectManager.hpp"
#include "Editor/ProjectResourceFactoryMP1.hpp"
#include "DataSpec/DNAMP1/Tweaks/CTweakGame.hpp"
#include "DataSpec/DNAMP1/Tweaks/CTweakPlayer.hpp"
#include "DataSpec/DNAMP1/Tweaks/CTweakPlayerControl.hpp"
#include "DataSpec/DNAMP1/Tweaks/CTweakGunRes.hpp"
#include "DataSpec/DNAMP1/Tweaks/CTweakPlayerRes.hpp"

namespace urde
{

namespace MP1
{

static logvisor::Module Log("MP1::CTweaks");

static const SObjectTag& IDFromFactory(CResFactory& factory, const char* name)
{
    const SObjectTag* tag = factory.GetResourceIdByName(name);
    if (!tag)
        Log.report(logvisor::Fatal, "Tweak Asset not found when loading... '%s'", name);
    return *tag;
}

void CTweaks::RegisterTweaks()
{
#if 0
    CResFactory& factory = *g_ResFactory;
    CResLoader& loader = factory.GetLoader();
    std::unique_ptr<CInputStream> strm;

    strm.reset(loader.LoadNewResourceSync(IDFromFactory(factory, "Game"), nullptr));
    TOneStatic<DataSpec::DNAMP1::CTweakGame> game(*strm);
    g_tweakGame = game.GetAllocSpace();
    strm.reset(loader.LoadNewResourceSync(IDFromFactory(factory, "Player"), nullptr));
    TOneStatic<DataSpec::DNAMP1::CTweakPlayer> player(*strm);
    g_tweakPlayer = player.GetAllocSpace();

    strm.reset(loader.LoadNewResourceSync(IDFromFactory(factory, "PlayerControls"), nullptr));
    TOneStatic<DataSpec::DNAMP1::CTweakPlayerControl> playerControl(*strm);
    g_tweakPlayerControl = playerControl.GetAllocSpace();
#endif
}

void CTweaks::RegisterResourceTweaks()
{
#if 0
    CResFactory& factory = *dynamic_cast<CResFactory*>(g_ResFactory);
    CResLoader& loader = factory.GetLoader();
    std::unique_ptr<CInputStream> strm;
    strm.reset(loader.LoadNewResourceSync(IDFromFactory(factory, "GunRes")));
    g_tweakGunRes = new DataSpec::DNAMP1::CTweakGunRes(*strm);
    strm.reset(loader.LoadNewResourceSync(IDFromFactory(factory, "PlayerRes"), nullptr));
    g_tweakPlayerRes = new DataSpec::DNAMP1::CTweakPlayerRes(*strm);
#else
    ProjectResourceFactoryMP1& factory = ProjectManager::g_SharedManager->resourceFactoryMP1();
    std::unique_ptr<CInputStream> strm;
    SObjectTag tag = *factory.GetResourceIdByName("GunRes");
    strm.reset(new CMemoryInStream(factory.LoadResourceSync(tag).release(), factory.ResourceSize(tag)));
    g_tweakGunRes = new DataSpec::DNAMP1::CTweakGunRes(*strm);
    tag = *factory.GetResourceIdByName("GunRes");
    strm.reset(new CMemoryInStream(factory.LoadResourceSync(tag).release(), factory.ResourceSize(tag)));
    g_tweakPlayerRes = new DataSpec::DNAMP1::CTweakPlayerRes(*strm);
#endif
}

}
}
