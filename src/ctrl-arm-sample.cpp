#include "Hubo_Control.h"
    
int main( int argc, char **argv )
{
    Hubo_Control hubo;
    
    // Vector6d is derived from the Matrix template in the Eigen C++ library
    // Vector6d is conveniently used for operations related to the arms and legs of Hubo because they are 6 DOF
    Vector6d q;
    
    // The left shift operator is a convenient way of setting the values of a Vector6d:
    q << -20.0/180.0*M_PI, 0, 0, -M_PI/2+20.0/180.0*M_PI, -0.3, 0;
    
    // This will tell the control daemon that you want the right arm joints to move to the positions specified by q:
    hubo.setRightArmAngles( q, true );
    // Setting the second argument to "true" makes it so the command is sent immediately instead of waiting until the end of a loop
    // Alternatively, you can say hubo.setRightArmAngles( q ) and it will assume "false" for the second argument as we will see in a later example
}
