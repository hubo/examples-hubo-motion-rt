#include <Hubo_Tech.h>

const double hipDistance = 0.08843*2.0; // Distance between hip joints

typedef enum {

    STATE_INVALID,
    S_HORSE,
    S_CRANE,
    Q_SHIFTDIST,
    Q_LIFTLEG,
    Q_CROUCH


} balance_state_t;


typedef enum {
    
    T_INVALID,
    T_INCOMPLETE,
    T_COMPLETE

} transition_result_t;




/**
 * The role of this class is to provide a
 * state machine and Kalman filter for
 * balancing and walking operations
*/
class Balance_Monitor
{

public:
    Balance_Monitor();
    

    Eigen::Vector2d shiftFilter( double FLsensor, double FRsensor, double dt );

    bool checkTransition( balance_state_t to ); // TODO: Fill this in all the way


    double hipVelocity; // TODO: Consider making this protected

protected:
    
    // Weight Shifting
    double FRprev;
    double FLprev;
    double Pprev;

    

    balance_state_t state;
 
    

    



};





// Stance Controllers
void calibrateBoth( Hubo_Tech &hubo );
void horseStance( Hubo_Tech &hubo );
void craneStance( int side, Hubo_Tech &hubo, double dt );



// Horse Stance Quasi-Statics
bool shiftToDistribution( int side, double distro, Hubo_Tech &hubo, Balance_Monitor &trans, double dt );
bool shiftToSide( int side, Hubo_Tech &hubo, Balance_Monitor &trans, double dt );
bool crouch( double height, Hubo_Tech &hubo, double dt );


// Crane Stance Quasi-Statics
bool liftLeg( int side, double height, Hubo_Tech &hubo, double dt );
bool placeSwingFoot( int side, Eigen::Vector3d footPose, Hubo_Tech &hubo, double dt );

