#include "Hubo_Control.h"
int main( int argc, char **argv )
{
    Hubo_Control hubo;     // This is the constructor for the hubo object


    double time, start;
    hubo.update(true);
    start = hubo.getTime();

    double cur = -0.12;

    bool moved[5] = {false, false, false, false, false};
    bool mode = false;

    int minute, phase;

    while(true)
    {
        hubo.update(true);

        time = hubo.getTime();
        minute = (int)((time-start)/10); // "minute" is a misnomer... it's actually 10 seconds
        phase = (int)minute/5;

        if( phase%2 == 0 && !mode )
        {
            cur *= -1;
            mode = true;
            fprintf( stdout, "Opening" );
            for(int i=0; i<5; i++)
                moved[i] = false;
        }
        else if( phase%2 == 1 && mode )
        {
            cur *= -1;
            mode = false;
            fprintf( stdout, "Closing" );
            for(int i=0; i<5; i++)
                moved[i] = false;
        }

        
        if( !moved[minute%5] )
        {
            switch( minute%5 )
            {
                case 0:
                    hubo.passJointAngle( RF1, cur );
                    hubo.passJointAngle( LF1, cur );
                    fprintf( stdout, " F1" );
                    moved[0] = true; break;
                case 1:
                    hubo.passJointAngle( RF2, cur );
                    hubo.passJointAngle( LF2, cur );
                    fprintf( stdout, ", F2" );
                    moved[1] = true; break;
                case 2:
                    hubo.passJointAngle( RF3, cur );
                    hubo.passJointAngle( LF3, cur );
                    fprintf( stdout, ", F3" );
                    moved[2] = true; break;
                case 3:
                    hubo.passJointAngle( RF4, cur );
                    hubo.passJointAngle( LF4, cur );
                    fprintf( stdout, ", F4" );
                    moved[3] = true; break;
                case 4:
                    hubo.passJointAngle( RF5, cur );
                    hubo.passJointAngle( LF5, cur );
                    fprintf(stdout, ", F5\n");
                    moved[4] = true; break;
            }
        }

        fflush(stdout);
        hubo.sendControls();
    }

/*
    float cur;

    
    if( argc < 2 )
        cur = 0;
    else
        cur = atof(argv[1]);

    if( cur > 0.1 )
        cur = 0.1;
    else if( cur < -0.1 )
        cur = -0.1;
    


    fprintf(stdout, "Setting current: %f\n", cur);

    hubo.setJointAngle(LEB, -M_PI/2);
    hubo.passJointAngle( RF1, cur, true );
*/
/*
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
*/
}
