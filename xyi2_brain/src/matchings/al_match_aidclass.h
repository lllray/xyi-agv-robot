#ifndef __al_match_aidclass_H
#define __al_match_aidclass_H

    #include <iostream>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    typedef class MATCH_PERFORMANCE
    {
    public:
        MATCH_PERFORMANCE()
        {
            issteady = true;
            posi_sigma = 0.0005f;
            angle_sigma = 0.015f;

            value_best = 1.0f;
            cov_posi_best = 10.0f;
            cov_angle_best = 10.0f;

            MCov.setZero();

            reset();
        }
        ~MATCH_PERFORMANCE()
        {

        }

        bool issteady;
        bool isgood;

        float values;
        float giveup_part;
        Eigen::Matrix3d MCov;

        void reset(void){cov_posi = 10, cov_angle = 10;
                         MCov.data()[0] = 10, MCov.data()[4] = 10, MCov.data()[8] = 10;
                         posimat_o.data()[0]= 1.0f, posimat_o.data()[5] = 1.0f, posimat_o.data()[10] = 1.0f, posimat_o.data()[15] = 1.0f;}

        void setdata(float value_in,
                     Eigen::Matrix4d &posimat_in,
                     Eigen::Matrix3f &Hessian_in)
        {
            #define value_terrible  0.5f
            #define k_posi_best 4.0f
            #define k_angle_best 7.5f
            #define min_protect 0.0001f
            #define valueserror_bound 0.005f
            #define valuessummat_bound 0.001f

            if(value_in < value_best)
                value_best = value_in;

            values = value_in;

            issteady = true;
            isgood = true;
            if(value_in > value_terrible)
                isgood = false;

            if( (std::abs(value_in - value_o) > valueserror_bound)      // isgood cond1: dvalue_bound is too large
              )
            {
                isgood = false;
                issteady = false;                                       // issteady cond1: dvalue_bound is too large
            }

            value_o = value_in;

            if(Hessian_in.data()[0] < min_protect) Hessian_in.data()[0] = min_protect;
            if(Hessian_in.data()[4] < min_protect) Hessian_in.data()[4] = min_protect;
            if(Hessian_in.data()[8] < min_protect) Hessian_in.data()[8] = min_protect;

            MCov.data()[0] = 1.0f / Hessian_in.data()[0];
            MCov.data()[4] = 1.0f / Hessian_in.data()[4];
            MCov.data()[8] = 1.0f / Hessian_in.data()[8];

            MCov.data()[0] *= posi_sigma;
            MCov.data()[4] *= posi_sigma;
            MCov.data()[8] *= angle_sigma;

            cov_posi = MCov.data()[0] + MCov.data()[4];
            cov_angle = MCov.data()[8];

            if(cov_posi < cov_posi_best)
                cov_posi_best = cov_posi;
            if(cov_angle < cov_angle_best)
                cov_angle_best = cov_angle;

            if( (cov_posi > (cov_posi_best * k_posi_best)) ||         // isgood cond2: cov_posi is too large
                (cov_angle > (cov_angle_best * k_angle_best))         // isgood cond3: cov_angle is too large
              )
            {
                isgood = false;
            }

            static int i;
            static Eigen::Matrix4d mat_d;
            mat_d = posimat_in - posimat_o;
            for(i=0, sum_dmat=0; i<16; i++)
                sum_dmat += std::abs(mat_d.data()[i]);

            if(sum_dmat >= valuessummat_bound)
                issteady = false;                                    // issteady cond2: sum(abs(d_mat)) is too large.

            posimat_o = posimat_in;


        }

        void show(void){ cout << "match_performance" << "[" << issteady << ", " << values << ", " << giveup_part << "]: "
                              << sum_dmat << ", " << value_best << endl; }

    private:
        float value_o, value_best, cov_posi_best, cov_angle_best;

        float sum_dmat;
        Eigen::Matrix4d posimat_o;

        float cov_posi, cov_angle;
        float posi_sigma, angle_sigma;

    }MATCH_PERFORMANCE;

    typedef struct ALM_NODE
    {
        Eigen::Vector3i node;
        float values;

        float gpart;

    }ALM_NODE;

#endif
