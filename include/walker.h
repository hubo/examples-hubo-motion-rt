#include <Hubo_Tech.h>





// Stance Controllers
void calibrateBoth( Hubo_Tech &hubo );
void horseStance( Hubo_Tech &hubo );
void craneStance( Hubo_Tech &hubo );


// Horse Stance Quasi-Statics
bool shiftToDistribution( int side, double distro, Hubo_Tech &hubo, double dt );
bool shiftToSide( int side, Hubo_Tech &hubo, double dt );
bool crouch( double height, Hubo_Tech &hubo, double dt );


// Crane Stance Quasi-Statics
bool liftLeg( int side, double height, Hubo_Tech &hubo, double dt );


