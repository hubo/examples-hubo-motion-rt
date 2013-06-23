#include <Hubo_Control.h>
#include <vector>
// Note that "std::vector" is a dynamic array class in C++ (not available in C)
// This means you can use std::vector to make a variable-sized array of ArmVector
// Even though std::vector and ArmVector have similar names, they are not directly related to each other
    
int main( int argc, char **argv )
{
    Hubo_Control hubo;

    std::vector<ArmVector, Eigen::aligned_allocator<ArmVector> > angles(5); // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5
    angles[0] <<   0.0556916,   0.577126,  0.0816814,  -0.492327, 0, 0, 0, 0, 0, 0;
    angles[1] <<  -1.07878,  0.408266, -0.477742, -0.665062, 0, 0, 0, 0, 0, 0;
    angles[2] <<   -1.17367, -0.0540511,  -0.772141,  -0.503859, 0, 0, 0, 0, 0, 0;
    angles[3] <<  -0.518417,   0.172191,  -0.566084, -0.0727671, 0, 0, 0, 0, 0, 0;
    angles[4] << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    // Obviously we could easily set up some code which reads in these values out of a pre-recorded file
    ArmVector currentPosition;  // This ArmVector will be used to track our current angle configuration

    ArmVector rangles;
    ArmVector accel, speeds;
    accel.setOnes(); accel *= 1.0;
    speeds.setOnes(); speeds *= 1.5;

    hubo.setLeftArmNomAcc( accel );
    hubo.setRightArmNomAcc( accel );
    hubo.setLeftArmNomSpeeds( speeds );
    hubo.setRightArmNomSpeeds( speeds );
    
    double tol = 0.075; // This will be the allowed tolerance before moving to the next point
    int traj = 0; // This will keep track of our current target on the trajectory
    while( true )
    {
        hubo.update(true); // This grabs the latest state information

        hubo.getLeftArmAngleStates( currentPosition ); // This will fill in the values of "currentPosition" by passing it in by reference
        // If you are unfamiliar with "passing by reference", let me know and I can explain the concept. It's a feature of C++ but not C
        
        if( ( currentPosition-angles[traj] ).norm() < tol ) // The class function .norm() is a feature of the Eigen libraries. Eigen has many other extremely useful functions like this.
        {
            traj++;
            if( traj > 4 )
                traj = 0;
        }
        
        rangles = angles[traj];

        rangles(1) = -rangles(1);
        rangles(2) = -rangles(2);
        hubo.setRightArmAngles( rangles, false );


        hubo.setLeftArmAngles( angles[traj] ); // Notice that the second argument is not passed in, making it default to "false"
        
        
        hubo.sendControls(); // This will send off all the latest control commands over ACH
    }
}
