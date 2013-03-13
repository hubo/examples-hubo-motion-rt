#include "Hubo_Control.h"
int main( int argc, char **argv )
{
    Hubo_Control hubo;     // This is the constructor for the hubo object
//    hubo.setJointAngle( REB, -M_PI/2, true ); // "true" instructs it to send the command right away. If you don't pass an argument here, the function assumes "false".
    hubo.update();
    Vector6d qdot;
    double time = hubo.getTime();
    double stime = hubo.getTime();

    while(true)
    {
        hubo.update(true);
        time = hubo.getTime();
//        if( hubo.getJointAngleState(RAP) > -30*M_PI/180.0 )
        if( time - stime > 7 )
        {
            Eigen::Vector3d vel, rot; vel.setZero(); rot.setZero();
            vel(0) = 0.05; vel(1) = -0.05;
            rot(2) = -5*M_PI/180.0;
            
            hubo.footVelocityIK( qdot, vel, rot, RIGHT );

            hubo.setRightLegVels( qdot, true );
        }
        else
        {
            qdot.setZero();
            hubo.setRightLegVels( qdot, true );
        }

    }

}
