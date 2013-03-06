#include <manip.h>

int main( int argc, char **argv )
{
    // Declare the manipulation command channel
    ach_channel_t chan_manip_cmd;

    // Open that channel
    int r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP, NULL );
    daemon_assert( r==ACH_OK, __LINE__ ); // Make sure it's okay

    hubo_manip_cmd manip; // Declare the manipulation struct
    size_t fs; // Placeholder

    // Grab the last frame which was used by the manipulator daemon to ensure consistency
    ach_get( &chan_manip_cmd, &manip, sizeof(manip), &fs, NULL, ACH_O_LAST );


    // Change the end effector destination in some way (You'll obviously want a loop here instead)
    manip.translation[RIGHT][0] = 0.359194;
    manip.translation[RIGHT][1] = -0.297988;
    manip.translation[RIGHT][2] = -0.15648;

    // Put the modified struct onto the channel
    ach_put( &chan_manip_cmd, &manip, sizeof(manip) );

    ach_close( &chan_manip_cmd );

}
