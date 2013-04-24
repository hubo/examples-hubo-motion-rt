#include <Hubo_Control>
#include "trajrun.h"

ach_channel_t traj_chan;
ach_channel_t traj_state_chan;

int main( int argc, char **argv )
{

    Hubo_Control hubo("trajrun");

    Vector6d rArmSpeedDef, lArmSpeedDef,
             rLegSpeedDef, lLegSpeedDef,
             rArmAccDef,   lArmAccDef,
             rLegAccDef,   lArmAccDef;

    hubo.getArmNomSpeed( RIGHT, rArmSpeedDef );
    hubo.getArmNomSpeed( LEFT, lArmSpeedDef );
    hubo.getLegNomSpeed( RIGHT, rLegSpeedDef );
    hubo.getLegNomSpeed( LEFT, lLegSpeedDef );

    hubo.getArmNomAcc( RIGHT, rArmAccDef );
    hubo.getArmNomAcc( LEFT, lArmAccDef );
    hubo.getLegNomAcc( RIGHT, rLegAccDef );
    hubo.getLegNomAcc( LEFT, lLegAccDef );
    
    hubo_traj_t traj;
    hubo_traj_output_t output;

    ach_open( &traj_chan, HUBO_TRAJ_CHAN, NULL );
    ach_open( &traj_state_chan, HUBO_TRAJ_STATE_CHAN, NULL );
    
    while( !sig_daemon_quit )
    {
        size_t fs;
        ach_result_t r;
        r = ach_get( &traj_chan, &traj, sizeof(traj), &fs, NULL, ACH_O_WAIT );
        if( ACH_OK != r )
            fprintf(stderr, "Warning: (%d) %s\n", (int)r, ach_result_to_string(r) );

        output.trajID = traj.trajID;
        output.status = TRAJ_RUNNING;
        ach_put( &traj_state_chan, &output, sizeof(output) );

        hubo.update(true);
        double clock = 0, start = hubo.getTime();
        int i=0;
        while( clock <= traj.endTime )
        {
            hubo.update(true);
            clock = hubo.getTime() - start;

            while( i<TRAJ_MAX_SIZE-1 && traj.time[i+1] <= clock
                                     && traj.time[i+1] != 0 )
                i++;

            for( int j=0; j < HUBO_JOINT_COUNT; j++ )
            {
                hubo.setJointAngle( j, traj.joint[j].position[i] );
                hubo.setJointNominalSpeed( j, traj.joint[j].velocity[i] );
                hubo.setJointNominalAcceleration( j, traj.joint[j].acceleration[i] );

                output.error[j] = traj.joint[j].position[i] - hubo.getJointAngleState(i);
            }
            
            hubo.sendControls();

        }
        
        output.status = TRAJ_COMPLETE;
        ach_put( &traj_state_chan, &output, sizeof(output) );
        
    }


    hubo.setArmNomSpeed( RIGHT, rArmSpeedDef );
    hubo.setArmNomSpeed( LEFT, lArmSpeedDef );
    hubo.setLegNomSpeed( RIGHT, rLegSpeedDef );
    hubo.setLegNomSpeed( LEFT, lLegSpeedDef );

    hubo.setArmNomAcc( RIGHT, rArmAccDef );
    hubo.setArmNomAcc( LEFT, lArmAccDef );
    hubo.setLegNomAcc( RIGHT, rLegAccDef );
    hubo.setLegNomAcc( LEFT, lLegAccDef );




}
