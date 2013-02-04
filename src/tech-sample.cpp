#include "Hubo_Tech.h"
int main( int argc, char **argv )
{
    Hubo_Tech hubo;     // This is the constructor for the hubo object
    hubo.setJointAngle( REB, -M_PI/2, true ); // "true" instructs it to send the command right away. If you don't pass an argument here, the function assumes "false".
}
