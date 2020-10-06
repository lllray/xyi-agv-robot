#ifndef __hfunc_H
#define __hfunc_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    // ***************************************************************************************************
    // >---------------------------------------------------------------------------------------------------
    // trifunc
    bool sin_chart_init(void);
    float m_sin(float dtain);
    float m_cos(float dtain);

    float m_rand(float b, float ed);
#endif

