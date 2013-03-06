#include "walker.h"


const double compRollGain = 0.0015;
const double compPitchGain = 0.0015;
const double shiftCompMultiplier = 1.0;
const double craneCompMultiplier  = 1.0;

const double craneShiftGainX = 0.0015; // Crane stance resistance to torque
const double craneShiftGainY = 0.0015; // Crane stance resistance to torque
const double craneHipPitchGain = 0.0*M_PI/180.0; // Not used
const double craneHipRollGain  = 0.05*M_PI/180.0; // Used when ankle roll reaches joint limits



const double pitchAngleGain = 0.5*M_PI/180.0; // IMU feedback gain
const double rollAngleGain = 0.3*M_PI/180.0; // IMU feedback gain
const double craneAngleMultiplier = 0.1;

const double shiftGain = 0.0002; // Shift-to-side gain
const double shiftBalanceGain = 0.0015; // Gain to maintain balance while shifting
const double craneTransitionPercent = 87.0; // Percent of weight on swing foot before entering crane stance

const double crouchGain = 0.30;  // Leg-lifting gain

const double pcAccel = 0.4; // Nominal acceleration used for position control
const double vcAccel = 1.5; // Nominal acceleration used for velocity control



void calibrateBoth( Hubo_Tech &hubo ) // Dynamic -- Precondition: Horse Stance
{
    double ptime, dt;

    double leftP =  0.0;
    double leftR =  0.0;
    double rightP = 0.0;
    double rightR = 0.0;

    double tol = 1e-5;

    hubo.update();
    ptime = hubo.getTime();

    bool starting = true;
    while( starting || (leftP*leftP + rightP*rightP
                    +leftR*leftR + rightR*rightR) > tol )
    {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if( dt > 0 )
        {
            starting = false;

            leftP =  pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy();
            leftR =  rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx();

            rightP = pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy();
            rightR = rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx();

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );

            hubo.sendControls();
        }
    }
    hubo.calibrateJoint( RAP );
    hubo.calibrateJoint( RAR );
    hubo.calibrateJoint( LAP );
    hubo.calibrateJoint( LAR );
    hubo.calibrateAnkleForces();
}

void horseStance( Hubo_Tech &hubo ) // Static Stance
{

    hubo.setJointVelocity( RAP, pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy() );
    hubo.setJointVelocity( RAR, rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx() );

    hubo.setJointVelocity( LAP, pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy() );
    hubo.setJointVelocity( LAR, rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx() );

    hubo.sendControls();

}

void craneStance( int side, Hubo_Tech &hubo, double dt )
{
    Vector6d swingVels;
    swingVels.setZero();
    craneStance( side, swingVels, hubo, dt );
}

void craneStance( int side, Vector6d swingVels, Hubo_Tech &hubo, double dt ) // Static Stance
{ 

    Eigen::Vector3d vel; vel.setZero();
    Vector6d rqvel, lqvel, acc; rqvel.setZero(); lqvel.setZero(); acc.setOnes();
    Vector6d q; q.setZero(); // TODO: Get rid of the need for this

    if( side == RIGHT )
    {
        lqvel = swingVels;

        vel(0) = craneShiftGainX*(  hubo.getRightFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getRightFootMx() );

        hubo.hipVelocityIK( rqvel, vel, RIGHT, q );

        rqvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX() 
                    + craneCompMultiplier*compRollGain*hubo.getRightFootMx();
        rqvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getRightFootMy();

        if(    hubo.getJointAngle(RAR)+rqvel(AR)*dt < hubo.getJointAngleMin(RAR)
            || (hubo.getJointAngleState(RHR) + hubo.getJointAngleState(RAR) < 0.0
                && hubo.getJointAngleState(RAR) < 0 && hubo.getJointAngleState(RHR) > 0) )
        {
            rqvel(AR) = 0.0;
            rqvel(HR) = craneHipRollGain*( -hubo.getRightFootMx() );
            lqvel(HR) += rqvel(HR);
        }
    
        hubo.setRightLegNomAcc( vcAccel*acc );
        
        if( hubo.getJointAngle(RHR)+rqvel(HR)*dt > hubo.getJointAngle(LHR)+lqvel(HR)*dt )
            lqvel(HR) = rqvel(HR);
    }
    else
    {
        rqvel = swingVels;

        vel(0) = craneShiftGainX*( hubo.getLeftFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getLeftFootMx() );

        hubo.hipVelocityIK( lqvel, vel, LEFT, q );
        
        lqvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX()
                    + craneCompMultiplier*compRollGain*hubo.getLeftFootMx();
        lqvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getLeftFootMy();

        if(    hubo.getJointAngle(LAR)+lqvel(AR)*dt > hubo.getJointAngleMax(LAR)
            || (hubo.getJointAngleState(LHR) + hubo.getJointAngleState(LAR) > 0.0
                && hubo.getJointAngleState(LAR) > 0 && hubo.getJointAngleState(LHR) < 0) )
        {
            lqvel(AR) = 0.0;
            lqvel(HR) = craneHipRollGain*( -hubo.getLeftFootMx() );
            rqvel(HR) += lqvel(HR);

            std::cout << "Crossed the boundary: " << hubo.getJointAngleState(LHR) << ", "
                    << hubo.getJointAngleState(LAR) << "\t:\t" << hubo.getJointAngle(LAR)+rqvel(AR)*dt
                    << ", " << hubo.getJointAngleMin(LAR) << std::endl;
        }

        hubo.setLeftLegNomAcc( vcAccel*acc );

        if( hubo.getJointAngle(LHR)+lqvel(HR)*dt < hubo.getJointAngle(RHR)*rqvel(HR)*dt )
            rqvel(HR) = lqvel(HR);
        
    }

    hubo.setLeftLegVels( lqvel );
    hubo.setRightLegVels( rqvel );

    

    hubo.setJointAngleMin( LHR, hubo.getJointAngleState(RHR)+rqvel(HR)*dt );
    hubo.setJointAngleMax( RHR, hubo.getJointAngleState(LHR)+lqvel(HR)*dt );
        

    hubo.sendControls();

}



/*
void craneStance( int side, Hubo_Tech &hubo, double dt ) // Static Stance
{ 

    Eigen::Vector3d vel; vel.setZero();
    Vector6d qvel, acc; qvel.setZero(); acc.setOnes();
    Vector6d q; q.setZero(); // TODO: Get rid of the need for this

    if( side == RIGHT )
    {
        vel(0) = craneShiftGainX*(  hubo.getRightFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getRightFootMx() );

        hubo.hipVelocityIK( qvel, vel, RIGHT, q );

        qvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX() 
                    + craneCompMultiplier*compRollGain*hubo.getRightFootMx();
        qvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getRightFootMy();

        if(    hubo.getJointAngle(RAR)+qvel(AR)*dt < hubo.getJointAngleMin(RAR)
            || (hubo.getJointAngleState(RHR) + hubo.getJointAngleState(RAR) < 0.0
                && hubo.getJointAngleState(RAR) < 0 && hubo.getJointAngleState(RHR) > 0) )
        {
            qvel(AR) = 0.0;
            qvel(HR) = craneHipRollGain*( -hubo.getRightFootMx() );
            hubo.setJointVelocity( LHR, qvel(HR) );
        }
        
        
        if( hubo.getJointAngle(RHR)+qvel(HR)*dt > hubo.getJointAngle(LHR) )
            hubo.setJointVelocity( LHR, qvel(HR) );

        hubo.setJointAngleMin( LHR, hubo.getJointAngleState(RHR)+qvel(HR)*dt );
    }
    else
    {
        vel(0) = craneShiftGainX*(  hubo.getLeftFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getLeftFootMx() );

        hubo.hipVelocityIK( qvel, vel, LEFT, q );
        
        qvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX()
                    + craneCompMultiplier*compRollGain*hubo.getLeftFootMx();
        qvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getLeftFootMy();

        if(    hubo.getJointAngle(LAR)+qvel(AR)*dt > hubo.getJointAngleMax(LAR)
            || (hubo.getJointAngleState(LHR) + hubo.getJointAngleState(LAR) > 0.0
                && hubo.getJointAngleState(LAR) > 0 && hubo.getJointAngleState(LHR) < 0) )
        {
            qvel(AR) = 0.0;
            qvel(HR) = craneHipRollGain*( -hubo.getLeftFootMx() );
            hubo.setJointVelocity( RHR, qvel(HR) );
        }


        if( hubo.getJointAngle(LHR)+qvel(HR)*dt < hubo.getJointAngle(RHR) )
            hubo.setJointVelocity( RHR, qvel(HR) );
        
        hubo.setJointAngleMax( RHR, hubo.getJointAngleState(LHR)+qvel(HR)*dt );
    }

    hubo.setLegVels( side, qvel );
    hubo.setLegNomAcc( side, vcAccel*acc );

    hubo.sendControls();

}
*/


bool shiftToDistribution( int side, double distro, Hubo_Tech &hubo, Balance_Monitor &trans, double dt )
{
    double tol = 0.001;


    Eigen::Vector3d vel;
    Eigen::Vector2d F;
    F = trans.shiftFilter( hubo.getLeftFootFz(), hubo.getRightFootFz(), dt );
    vel << 0.00, 0.00, 0.00;

    Vector6d rqvel, lqvel, acc; acc.setOnes();
    Vector6d q; q.setZero(); // TODO: Get rid of the need
    
    

    if( side == RIGHT )
        vel(1) = -shiftGain*( F(LEFT) - (100.0-distro)/100.0*(F(RIGHT)+F(LEFT)) );
    else
        vel(1) = shiftGain*( F(RIGHT) - (100.0-distro)/100.0*(F(RIGHT)+F(LEFT)) );

    vel(0) = shiftBalanceGain*( hubo.getLeftFootMy() + hubo.getRightFootMy() );
    // TODO: Deal with staggered stance

    trans.hipVelocity = vel(1);

    hubo.hipVelocityIK( lqvel, vel, LEFT, q );
    hubo.setLeftLegVels( lqvel );
    hubo.setLeftLegNomAcc( vcAccel*acc );

    hubo.hipVelocityIK( rqvel, vel, RIGHT, q );
    hubo.setRightLegVels( rqvel );
    hubo.setRightLegNomAcc( vcAccel*acc );

    std::cout << "RFz: " << F(RIGHT) << "aRFz: " << hubo.getRightFootFz() << "\t\tLFz: " << F(LEFT)
                << "aLFz: " << hubo.getLeftFootFz() << "\t\tLMx: " << hubo.getLeftFootMx()
                << "\t\tVel: " << vel(1) << std::endl;


    lqvel(AR) += rollAngleGain*hubo.getAngleX()  + shiftCompMultiplier*compRollGain*hubo.getLeftFootMx();
    lqvel(AP) += pitchAngleGain*hubo.getAngleY() + shiftCompMultiplier*compPitchGain*hubo.getLeftFootMy();

    rqvel(AR) += rollAngleGain*hubo.getAngleX()  + shiftCompMultiplier*compRollGain*hubo.getRightFootMx();
    rqvel(AP) += pitchAngleGain*hubo.getAngleY() + shiftCompMultiplier*compPitchGain*hubo.getRightFootMy();


    hubo.setJointAngleMin( LHR, hubo.getJointAngleState(RHR)+rqvel(1)*dt );
    hubo.setJointAngleMax( RHR, hubo.getJointAngleState(LHR)+lqvel(1)*dt );

    hubo.sendControls();

    if( fabs(vel(1)) < tol )
        return true;
    else
        return false;
}

bool shiftToSide( int side, Hubo_Tech &hubo, Balance_Monitor &trans, double dt )
{// Quasi-Static -- Precondition: Horse Stance
 // Transition: Horse Stance >>> Crane Stance

    return shiftToDistribution( side, craneTransitionPercent, hubo, trans, dt );
}

bool crouch( double height, Hubo_Tech &hubo, double dt ) // Quasi-Static -- Precondition: Horse Stance
{

    Eigen::Isometry3d B;
    Vector6d legAngles, rqvel, lqvel, acc; acc.setZero();
    Vector6d q; q.setZero(); // TODO: Make this unnecessary

    double tol = 0.001;

    int side;
    if( hubo.getRightFootFz() > hubo.getLeftFootFz() )
        side = RIGHT;
    else
        side = LEFT;

    Eigen::Vector3d vel;
    vel << 0.00, 0.00, 0.00;

    hubo.getLegAngles( side, legAngles );
    hubo.huboLegFK( B, legAngles, side );

    vel(2) = crouchGain*( height + B(2,3) ); // This is added because B(2,3) measures from the neck to
                                             // the foot and therefore will be negative

    hubo.hipVelocityIK( lqvel, vel, LEFT, q );
    hubo.setLeftLegVels( lqvel );
    hubo.setLeftLegNomAcc( vcAccel*acc );

    hubo.hipVelocityIK( rqvel, vel, RIGHT, q );
    hubo.setRightLegVels( rqvel );
    hubo.setRightLegNomAcc( vcAccel*acc );

    lqvel(AR) += rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx();
    lqvel(AP) += pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy();

    rqvel(AR) += rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx();
    rqvel(AP) += pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy();
    
    hubo.setJointAngleMin( LHR, hubo.getJointAngleState(RHR)+rqvel(HR)*dt );
    hubo.setJointAngleMax( RHR, hubo.getJointAngleState(LHR)+lqvel(HR)*dt );

    hubo.sendControls();

    if( fabs(vel(2)) < tol )
        return true;
    else
        return false;
}


bool placeSwingFoot( int side, Eigen::Vector3d footPose, Hubo_Tech &hubo, double dt ) // Quasi-Static -- Precondition: Crane Stance on opposite leg
{
    int stance = abs(side-1); // Stance leg will be the opposite leg
    double tol = 0.001;

    Eigen::Isometry3d S, B;
    Vector6d legAngles, stanceAngles, qvel; qvel.setZero();
    Vector6d q, acc; q.setZero(); acc.setOnes();

    Eigen::Vector3d vel;
    vel.setZero();

    hubo.getLegAngles( side, legAngles );
    hubo.huboLegFK( B, legAngles, side );
    hubo.getLegAngles( stance, stanceAngles );
    hubo.huboLegFK( S, stanceAngles, stance );

//  **** Velocity-based (produces high acceleration)
    vel = -crouchGain*( footPose - (B.translation() - S.translation()) );
    // B -S represents the swing foot location in the stance foot frame
    // Note: The negative sign is because the hipVelocity IK is going to be used.
    //       Hip velocities are always identically the opposite of foot velocities.


    hubo.hipVelocityIK( qvel, vel, side, q );
//    hubo.setLegVels( side, qvel );
    hubo.setLegNomAcc( side, pcAccel*acc );

    
    craneStance( stance, qvel, hubo, dt ); // Controls get sent in the craneStance function

    if( vel.norm() < tol )
        return true;
    else
        return false;
    
}





int main(int argc, char **argv)
{
    Hubo_Tech hubo;
    Balance_Monitor trans;

    calibrateBoth(hubo);

    
    Eigen::Vector3d placeFoot;
    placeFoot << 0, -0.08843*2.0, 0.15;


    hubo.update();
    double dt, stime = hubo.getTime();
    double time = stime;

    bool ready = false;
    bool go = false;

    stime = hubo.getTime();
    time = hubo.getTime();

    while( true ) //time-stime < 5 )
    {

        hubo.update(true);
        dt = hubo.getTime() - time;
        time = hubo.getTime();
        

        if( dt > 0 )
        {


//            liftLeg( RIGHT, 0.15, hubo, dt );
/*            
            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else if(!go)
                go = placeSwingFoot( RIGHT, placeFoot, hubo, dt );
            else
                craneStance( LEFT, hubo, dt );
*/

//            shiftToSide( LEFT, hubo, trans, dt );

//          *** Shifting weight endurance test
/*
            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else
                ready = !shiftToSide( RIGHT, hubo, trans, dt );
*/

//          *** Standard shift & crane


            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else if(!go)
                go = placeSwingFoot( RIGHT, placeFoot, hubo, dt );
            else
                craneStance( LEFT, hubo, dt );

        }
    }



}
