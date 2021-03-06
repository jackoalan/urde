include_directories(../hecl/include
                    ../hecl/extern/boo/include
                    ../hecl/extern/athena/include
                    ../specter/zeus/include
                    ../nod/include
                    ../nod/logvisor/include
                    ../nod/logvisor/fmt/include
                    ../DataSpec
                    ../Runtime
                    ..)
add_compile_definitions(ZE_ATHENA_TYPES=1)
add_shader(CAABoxShader)
add_shader(CCameraBlurFilter)
add_shader(CColoredQuadFilter)
add_shader(CColoredStripShader)
add_shader(CDecalShaders)
add_shader(CElementGenShaders)
add_shader(CEnergyBarShader)
add_shader(CEnvFxShaders)
add_shader(CFogVolumeFilter)
add_shader(CFogVolumePlaneShader)
add_shader(CLineRendererShaders)
add_shader(CMapSurfaceShader)
add_shader(CMoviePlayerShader)
add_shader(CNESShader)
add_shader(CParticleSwooshShaders)
add_shader(CPhazonSuitFilter)
add_shader(CRadarPaintShader)
add_shader(CRandomStaticFilter)
add_shader(CScanLinesFilter)
add_shader(CSpaceWarpFilter)
add_shader(CTextSupportShader)
add_shader(CTexturedQuadFilter)
add_shader(CThermalColdFilter)
add_shader(CThermalHotFilter)
add_shader(CWorldShadowShader)
add_shader(CXRayBlurFilter)
add_special_shader(shader_CFluidPlaneShader
                   shader_CFluidPlaneShader.cpp
                   shader_CFluidPlaneShaderGLSL.cpp
                   shader_CFluidPlaneShaderHLSL.cpp
                   shader_CFluidPlaneShaderMetal.cpp)
add_special_shader(shader_CModelShaders
                   shader_CModelShadersGLSL.cpp
                   shader_CModelShadersHLSL.cpp
                   shader_CModelShadersMetal.cpp)