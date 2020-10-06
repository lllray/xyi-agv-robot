/* ********************************************************************************
 * author: chenyingbing
 * time: 20170512   14:55   in XIAMEN University
 * illustration:
 *      the param and data init part.
 *
 * *******************************************************************************/

#include "struct_map.h"

void Struct_Map::Map_PramInit(float *param, float _mdismap_range)
{
    sigama = param[0];
    Nmap_sigama = 1.0f / ( 2 * param[0] * param[0]);

    powlreso_nigama[0] = pow_lreso[0] * Nmap_sigama;
    powlreso_nigama[1] = pow_lreso[1] * Nmap_sigama;

    mdismap_range = _mdismap_range;

    param_wait = false;
}

void Struct_Map::Map_Alloc_Init(void)
{
    if(!map_isalloc)
    {
        float d1, d2;
        for(char i=0; i<2; i++)
        {
            /// Common Part

            /// >-------------------------------------------------------------------------------
            /// OccupiedGrid Map
            lmap[i].header.frame_id = "/map_r";

            lmap[i].info.resolution = lreso[i];
            lmap[i].info.width = lwidth[i];
            lmap[i].info.height = lheight[i];
            lsize[i] = lwidth[i] * lheight[i];

            lmap[i].info.origin.orientation.w = 1.0f,
             lmap[i].info.origin.orientation.x = 0.0,
              lmap[i].info.origin.orientation.y = 0.0,
               lmap[i].info.origin.orientation.z = 0.0;

            d1 = (lwidth[i] * 0.5f);
            d2 = (lheight[i] * 0.5f);

            lmap[i].info.origin.position.x = -(d1 + 0.5f) * lreso[i];
            lmap[i].info.origin.position.y = -(d2 + 0.5f) * lreso[i];
            lmap[i].info.origin.position.z = 0;

            map_origin[i] = lmap[i].info.origin;
            map_xiuz[i].position.x = -map_origin[i].position.x;
            map_xiuz[i].position.y = -map_origin[i].position.y;

            lmap[i].data.resize(lsize[i], Mnode_Unkown);
            lnmap[i].resize(lsize[i], 0);
            mdismap[i].resize(lsize[i], Disnode_Unkown);

        }

        /// >-------------------------------------------------------------------------------
        /// Common Part
        map_isalloc = true;
    }
}

bool Struct_Map::Map_alloc_sta(void)
{
    return map_isalloc;
}
