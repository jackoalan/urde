make_dnalist(PAK
             MLVL
             ANIM
             CHAR
             CMDLMaterials
             CINF
             CSKR
             MREA
             SAVW
             CAUD
             HINT
             SAND)
set(DNAMP3_SOURCES
    DNAMP3.hpp DNAMP3.cpp
    PAK.cpp
    ANIM.cpp
    CINF.cpp
    CHAR.cpp
    CMDL.hpp CMDL.cpp
    CMDLMaterials.cpp
    CSKR.cpp
    STRG.hpp STRG.cpp
    MAPA.hpp
    MREA.cpp)

dataspec_add_list(DNAMP3 DNAMP3_SOURCES)
