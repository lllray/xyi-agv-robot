#include "r_controller.h"

// when return true, data == 0;
static bool protectmin_value(float &data, float abs_limit)
{
    if(std::abs(data) < abs_limit)
    {
        if(data > 0)
            data = abs_limit;
        else if(data < 0)
            data = -abs_limit;
        else
            return true;
    }

    return false;
}

static void protectmax_value(float &data, float abs_limit)
{
    if(data > abs_limit) data = abs_limit;
    if(data < -abs_limit) data = -abs_limit;
}

// ************************************************************************************

void r_controller::stop_control(void)
{
    controlcmd_pub(0, 0);
}

void r_controller::acelelimit_control(float control_T,
                                      Eigen::Vector2f _target_vgyro)
{
    static float dv_limit, dgyro_limit;
    static float dv, dgyro;

    dv_limit = MAX_VELO_ACCELERATION * control_T;
    dgyro_limit = MAX_GYRO_ACCELERATION * control_T;

    dv = _target_vgyro(0) - state_v;
    dgyro = _target_vgyro(1) - state_gyro;

    if(std::abs(dv) > dv_limit)
        _target_vgyro(0) = state_v + ( dv >= 0 ? dv_limit : -dv_limit);

    if(std::abs(dgyro) > dgyro_limit)
        _target_vgyro(1) = state_gyro + ( dgyro >= 0 ? dgyro_limit : -dgyro_limit);

    protectmax_value(_target_vgyro(0), MAX_VELO);
    protectmax_value(_target_vgyro(1), MAX_GYRO);

    controlcmd_pub(_target_vgyro(0), _target_vgyro(1));
}

void r_controller::instant_control(Eigen::Vector2f _target_vgyro)
{
    protectmax_value(_target_vgyro(0), MAX_VELO);
    protectmax_value(_target_vgyro(1), MAX_GYRO);

    controlcmd_pub(_target_vgyro(0), _target_vgyro(1));
}

// ************************************************************************************

void r_controller::controlcmd_pub(float command_v, float command_gyro)
{
    #define min_control_T   0.1f

    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<geometry_msgs::Twist>("/RobotPort_CmdVel",1);
    static geometry_msgs::Twist twistdta;

    twistdta.linear.x = command_v,
     twistdta.linear.y = 0,
      twistdta.linear.z = 0;

    twistdta.angular.x = 0,
     twistdta.angular.y = 0,
      twistdta.angular.z = command_gyro;

    state_v = command_v;
    state_gyro = command_gyro;

    datapub.publish(twistdta);
}

// ************************************************************************************

