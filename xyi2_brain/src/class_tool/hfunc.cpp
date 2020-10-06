#include "hfunc.h"

#include "tfm.h"

#define PI           3.1415926536f
#define PI_2         1.5707963268f
#define C_RADIAN     57.2957795f
#define _C_RADIAN    0.0174533f

#define sin_chart_d     0.25f
static bool chart_init = false;
static double sin_chart[361];

// ***************************************************************************************************
// >---------------------------------------------------------------------------------------------------
// trifunc

bool sin_chart_init(void)
{
    const float d1 = 0.25f / C_RADIAN;
    if(!chart_init)
    {
        int i;
        float fi;
        for(i=0, fi=0.0f; i<361; i++)
        {
            fi = (float)i * d1;
            sin_chart[i] = std::sin(fi);
        }

        chart_init = true;
    }
    return chart_init;
}

float m_sin(float dtain)
{
    if( (0<=dtain) && (dtain<=PI_2) )
    {
        dtain *= C_RADIAN;

        float d1, d2;
        d1 = std::floor(dtain * 4) * 0.25f;
        d2 = dtain - d1;

        int di;
        di = d1 * 4;

        d1 = sin_chart[di];
        if(di < 360)
            d2 = (sin_chart[di+1]-sin_chart[di]) * d2;
        else
            d2 = 0;

        return (d1+d2);
    }else if(dtain > PI_2)
    {
        return m_sin(PI - dtain);
    }else{
        dtain *= -1;
        return -m_sin(dtain);
    }
}

float m_cos(float dtain)
{
    dtain = dtain + PI_2;
    if(dtain > PI) dtain = -2*PI + dtain;

    return m_sin(dtain);
}

float m_rand(float b, float ed)
{
    static float value;

    std::srand((unsigned int)std::time(0));

    value = std::rand();
    value /= RAND_MAX;      /// 0 ~ 1.0f

    value *= (ed - b);      /// 0 ~ (ed-b);
    value += b;             /// b ~ ed

    return value;
}

// ***************************************************************************************************
// >---------------------------------------------------------------------------------------------------
// pro_map func


