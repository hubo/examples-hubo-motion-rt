#include "walker.h"
#include <hubo-zmp.h>


ach_channel_t zmp_chan;



const double nudgePGain = 0.04;
const double nudgeIGain = 0.70;
const double nudgeDGain = 0.80;



const double compRollGain = 0.0015;
const double compPitchGain = 0.0015;
const double shiftCompMultiplier = 1.0;
const double craneCompMultiplier = 1.0;

const double craneShiftGainX = 0.0015; // Crane stance resistance to torque
const double craneShiftGainY = 0.0015; // Crane stance resistance to torque
const double craneHipPitchGain = 0.0*M_PI/180.0; // Not used
const double craneHipRollGain  = 0.05*M_PI/180.0; // Used when ankle roll reaches joint limits
const double craneAnkleRollThresh = 2.0*M_PI/180.0;

const double craneHipSafetyMargin = 2.5*M_PI/180.0; // Buffer to prevent leg collisions during crane stance
const double craneHipSafetyGain = 1.5;

const double swingVelLimit = 0.05;

const double pitchAngleGain = 0.5*M_PI/180.0; // IMU feedback gain
const double rollAngleGain = 0.3*M_PI/180.0; // IMU feedback gain
const double craneAngleMultiplier = 1.0;

const double shiftGain = 0.0002; // Shift-to-side gain
const double shiftBalanceGain = 0.0015; // Gain to maintain balance while shifting
const double craneTransitionPercent = 98.0; // Percent of weight on swing foot before entering crane stance

const double crouchGain = 0.30;  // Leg-lifting gain

const double pcAccel = 0.4; // Nominal acceleration used for position control
const double vcAccel = 1.5; // Nominal acceleration used for velocity control

const double calKneeGain = 1.0;
const double calWeightGain = 0.001; // rad/sec per N


void calibrateBoth( Hubo_Control &hubo ) // Dynamic -- Precondition: Horse Stance
{
    double ptime, dt;

    double kneeEq = 0.2;

    double leftP =  0.0;
    double leftR =  0.0;
    double rightP = 0.0;
    double rightR = 0.0;

    double leftK  = 0.0;
    double rightK = 0.0;

    double tol = 1e-9;

    hubo.update();
    ptime = hubo.getTime();

    double defaultLHPLimit = hubo.getJointAngleMax(LHP);
    double defaultRHPLimit = hubo.getJointAngleMax(RHP);

    hubo.setJointAngleMax( LHP, 0 );
    hubo.setJointAngleMax( RHP, 0 );

    bool starting = true;
    while( starting || (leftP*leftP + rightP*rightP
                    +leftR*leftR + rightR*rightR) > tol*100000 )
    {
        hubo.update(true);
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if( dt > 0 )
        {
            starting = false;

            leftP = -leftK/2.0 + pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy();
            leftR = rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx();

            rightP = -rightK/2.0 + pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy();
            rightR = rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx();

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );

            hubo.sendControls();
        }
    }
    starting = true;
    while( starting || ( leftK*leftK + rightK*rightK ) > tol )
    {
        hubo.update(true);
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if( dt > 0 )
        {
            starting = false;

            leftK =  calKneeGain*( kneeEq - hubo.getJointAngleState(LKN) ) + calWeightGain*( hubo.getLeftFootFz() - hubo.getRightFootFz() );
            rightK = calKneeGain*( kneeEq - hubo.getJointAngleState(RKN) ) + calWeightGain*( hubo.getRightFootFz() - hubo.getLeftFootFz() );


            leftP = -leftK/2.0 + pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy();
            leftR = rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx();

            rightP = -rightK/2.0 + pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy();
            rightR = rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx();

            fprintf(stdout, "LFz: %3.2f \t RFz: %3.2f \t Error: %2.2f \t LK: %1.6f \t RK: %1.6f\n", hubo.getLeftFootFz(), hubo.getRightFootFz(), hubo.getLeftFootFz()-hubo.getRightFootFz(), leftK, rightK );
            
            hubo.setJointVelocity( RKN, rightK );
            hubo.setJointVelocity( RHP, -rightK/2.0 );
            
            hubo.setJointVelocity( LKN, leftK );
            hubo.setJointVelocity( LHP, -leftK/2.0 );

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );

            hubo.sendControls();
        }
    }
    hubo.calibrateJoint( RAP, kneeEq );
    hubo.calibrateJoint( RAR );
    hubo.calibrateJoint( LAP, kneeEq );
    hubo.calibrateJoint( LAR );
//    hubo.calibrateAnkleForces();

    hubo.setJointAngleMax( LHP, defaultLHPLimit );
    hubo.setJointAngleMax( RHP, defaultRHPLimit );
}

void horseStance( Hubo_Control &hubo ) // Static Stance
{

    hubo.setJointVelocity( RAP, pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getRightFootMy() );
    hubo.setJointVelocity( RAR, rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getRightFootMx() );

    hubo.setJointVelocity( LAP, pitchAngleGain*hubo.getAngleY() + compPitchGain*hubo.getLeftFootMy() );
    hubo.setJointVelocity( LAR, rollAngleGain*hubo.getAngleX()  + compRollGain*hubo.getLeftFootMx() );

    hubo.sendControls();

}

void craneStance( int side, Hubo_Control &hubo, double dt )
{
    Vector6d swingVels;
    swingVels.setZero();
    craneStance( side, swingVels, hubo, dt );
}

void craneStance( int side, Vector6d swingVels, Hubo_Control &hubo, double dt ) // Static Stance
{ 

    Eigen::Vector3d vel; vel.setZero();
    Vector6d rqvel, lqvel, acc; rqvel.setZero(); lqvel.setZero(); acc.setOnes();
    Vector6d q; q.setZero(); // TODO: Get rid of the need for this

    if( side == RIGHT )
    {
        lqvel = swingVels;

        vel(0) = craneShiftGainX*(  hubo.getRightFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getRightFootMx() );

        hubo.hipVelocityIK( rqvel, vel, RIGHT );

        rqvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX() 
                    + craneCompMultiplier*compRollGain*hubo.getRightFootMx();
        rqvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getRightFootMy();

//        if(    hubo.getJointAngle(RAR)+rqvel(AR)*dt < hubo.getJointAngleMin(RAR)
        if(    hubo.getJointAngle(RAR) <= hubo.getJointAngleMin(RAR)+craneAnkleRollThresh
            || (hubo.getJointAngleState(RHR) + hubo.getJointAngleState(RAR) < 0.0
                && hubo.getJointAngleState(RAR) < 0 && hubo.getJointAngleState(RHR) > 0) )
        {
            rqvel(AR) = 0.0;
            rqvel(HR) = craneHipRollGain*( -hubo.getRightFootMx() );
            lqvel(HR) += rqvel(HR);
        }
    
        hubo.setRightLegNomAcc( vcAccel*acc );
        
        if( (hubo.getJointAngle(RHR)+rqvel(HR)*dt) - (hubo.getJointAngle(LHR)+lqvel(HR)*dt) > -craneHipSafetyMargin )
            lqvel(HR) -= craneHipSafetyGain*( (hubo.getJointAngle(RHR)+rqvel(HR)*dt)
                            - (hubo.getJointAngle(LHR)+lqvel(HR)*dt) + craneHipSafetyMargin );

//        lqvel(AR) = 0.0;
//        lqvel(AP) = 0.0;
    }
    else
    {
        rqvel = swingVels;

        vel(0) = craneShiftGainX*( hubo.getLeftFootMy() );
        vel(1) = craneShiftGainY*( -hubo.getLeftFootMx() );

        hubo.hipVelocityIK( lqvel, vel, LEFT );
        
        lqvel(AR) += craneAngleMultiplier*rollAngleGain*hubo.getAngleX()
                    + craneCompMultiplier*compRollGain*hubo.getLeftFootMx();
        lqvel(AP) += craneAngleMultiplier*pitchAngleGain*hubo.getAngleY()
                    + craneCompMultiplier*compPitchGain*hubo.getLeftFootMy();

//        if(    hubo.getJointAngle(LAR)+lqvel(AR)*dt > hubo.getJointAngleMax(LAR)
        if(    hubo.getJointAngle(LAR) > hubo.getJointAngleMax(LAR)-craneAnkleRollThresh
            || (hubo.getJointAngleState(LHR) + hubo.getJointAngleState(LAR) > 0.0
                && hubo.getJointAngleState(LAR) > 0 && hubo.getJointAngleState(LHR) < 0) )
        {
            lqvel(AR) = 0.0;
            lqvel(HR) = craneHipRollGain*( -hubo.getLeftFootMx() );
            rqvel(HR) += lqvel(HR);


//            std::cout << "Crossed Border: " << hubo.getJointAngle(LAR)+lqvel(AR)*dt << ":" << hubo.getJointAngleMax(LAR)
//                << "\t" << hubo.getJointAngleState(LHR) << "," << hubo.getJointAngleState(LAR) << std::endl;
        }

        hubo.setLeftLegNomAcc( vcAccel*acc );

        if( (hubo.getJointAngle(LHR)+lqvel(HR)*dt) - (hubo.getJointAngle(RHR)+rqvel(HR)*dt) < craneHipSafetyMargin )
        {
            rqvel(HR) += craneHipSafetyGain*( (hubo.getJointAngle(LHR)+lqvel(HR)*dt) - (hubo.getJointAngle(RHR)+rqvel(HR)*dt) - craneHipSafetyMargin );
            std::cout << 180.0/M_PI*((hubo.getJointAngle(LHR)+lqvel(HR)*dt) - (hubo.getJointAngle(RHR)+rqvel(HR)*dt) - craneHipSafetyMargin) << std::endl;
        }
        
//        rqvel(AR) = 0.0;
//        rqvel(AP) = 0.0;
//        if( hubo.getJointAngle(LHR)+lqvel(HR)*dt+craneHipSafetyMargin < hubo.getJointAngle(RHR)*rqvel(HR)*dt )
//            rqvel(HR) = lqvel(HR);
        
    }

    hubo.setLeftLegVels( lqvel );
    hubo.setRightLegVels( rqvel );

    

    hubo.setJointAngleMin( LHR, hubo.getJointAngleState(RHR)+rqvel(HR)*dt );
    hubo.setJointAngleMax( RHR, hubo.getJointAngleState(LHR)+lqvel(HR)*dt );
        

    hubo.sendControls();

}


bool shiftToDistribution( int side, double distro, Hubo_Control &hubo, Balance_Monitor &trans, double dt )
{
    double tol = 0.001;


    Eigen::Vector3d vel;
    Eigen::Vector2d F;
//    F = trans.shiftFilter( hubo.getLeftFootFz(true), hubo.getRightFootFz(true), dt );
    F(LEFT) = hubo.getLeftFootFz();
    F(RIGHT) = hubo.getRightFootFz();
    vel << 0.00, 0.00, 0.00;

    Vector6d rqvel, lqvel, acc; acc.setOnes();
    Vector6d q; q.setZero(); // TODO: Get rid of the need
    
    

    if( side == RIGHT )
        vel(1) = -shiftGain*( F(LEFT) - (100.0-distro)/100.0*(F(RIGHT)+F(LEFT)) );
    else
        vel(1) = shiftGain*( F(RIGHT) - (100.0-distro)/100.0*(F(RIGHT)+F(LEFT)) );

    if( fabs(vel(1)) > 0.5 )
        vel(1) = 0.5*vel(1)/fabs(vel(1));

    vel(0) = shiftBalanceGain*( hubo.getLeftFootMy() + hubo.getRightFootMy() );
    // TODO: Deal with staggered stance
    
    ctrl_flag_t likresult = hubo.hipVelocityIK( lqvel, vel, LEFT );
    ctrl_flag_t rikresult = hubo.hipVelocityIK( rqvel, vel, RIGHT );

    if( likresult!=IK_DANGER && rikresult!=IK_DANGER )
        trans.hipVelocity = vel(1);

    hubo.setLeftLegVels( lqvel );
    hubo.setLeftLegNomAcc( vcAccel*acc );

    hubo.setRightLegVels( rqvel );
    hubo.setRightLegNomAcc( vcAccel*acc );

    std::cout << "RFz: " << F(RIGHT) << "aRFz: " << hubo.getRightFootFz(true) << "\t\tLFz: " << F(LEFT)
                << "aLFz: " << hubo.getLeftFootFz(true) << "\t\tLMx: " << hubo.getLeftFootMx()
                << "\t\tVel: " << vel(1) << "\tError:" << fabs(F(RIGHT)-hubo.getRightFootFz(true))+fabs(F(LEFT)-hubo.getLeftFootFz(true)) << std::endl;


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

bool shiftToSide( int side, Hubo_Control &hubo, Balance_Monitor &trans, double dt )
{// Quasi-Static -- Precondition: Horse Stance
 // Transition: Horse Stance >>> Crane Stance

    return shiftToDistribution( side, craneTransitionPercent, hubo, trans, dt );
}

bool crouch( double height, Hubo_Control &hubo, double dt ) // Quasi-Static -- Precondition: Horse Stance
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

    hubo.hipVelocityIK( lqvel, vel, LEFT );
    hubo.setLeftLegVels( lqvel );
    hubo.setLeftLegNomAcc( vcAccel*acc );

    hubo.hipVelocityIK( rqvel, vel, RIGHT );
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


bool placeSwingFoot( int side, Eigen::Vector3d footPose, Hubo_Control &hubo, double dt ) // Quasi-Static -- Precondition: Crane Stance on opposite leg
{
    int stance = abs(side-1); // Stance leg will be the opposite leg
    double tol = 0.01;

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
    if( vel.norm() > swingVelLimit )
        vel = vel.normalized()*swingVelLimit;


    hubo.hipVelocityIK( qvel, vel, side );
//    hubo.setLegVels( side, qvel );
    hubo.setLegNomAcc( side, pcAccel*acc );

    
    craneStance( stance, qvel, hubo, dt ); // Controls get sent in the craneStance function

    if( vel.norm() < tol )
        return true;
    else
        return false;
    
}

void dualLegNudge( Hubo_Control &hubo, const zmp_traj_element_t &elem, 
                      Vector6d &lqdot, Vector6d &rqdot,
                      foot_state_t &state, double dt )
//                    Eigen::Vector3d &vprev,
//                    Eigen::Vector3d &verr, double dt )
{
    std::cout << "Dual Leg: ";

    Eigen::Isometry3d Br, Bl;
    Eigen::Vector3d ahat;
    Vector6d rangles, langles;

    hubo.getLeftLegAngles( langles );
    hubo.huboLegFK( Bl, langles, LEFT );
    hubo.getRightLegAngles( rangles );
    hubo.huboLegFK( Br, rangles, RIGHT );

    ahat = (Br.translation()-Bl.translation()).normalized();

    Vector3d Ml, Mr, Ma, vel; Ml.setZero(); Mr.setZero(); Ma.setZero();

    Mr(0) = hubo.getRightFootMx()*cos(hubo.getJointAngleState(RHY))
            - hubo.getRightFootMy()*sin(hubo.getJointAngleState(RHY));
            - elem.torque[RIGHT][0]; // FIXME: Use the correct formatting here
    Mr(1) = hubo.getRightFootMx()*sin(hubo.getJointAngleState(RHY))
            + hubo.getRightFootMy()*cos(hubo.getJointAngleState(RHY));
            - elem.torque[RIGHT][1];
    Ml(0) = hubo.getLeftFootMx()*cos(hubo.getJointAngleState(LHY))
            - hubo.getLeftFootMy()*sin(hubo.getJointAngleState(LHY));
            - elem.torque[LEFT][0];
    Ml(1) = hubo.getLeftFootMx()*sin(hubo.getJointAngleState(LHY))
            + hubo.getLeftFootMy()*cos(hubo.getJointAngleState(LHY));
            - elem.torque[LEFT][1];

    Ma = (Ml+Mr).dot(ahat)*ahat;

    std::cout << "Ma:" << Ma.transpose();// << "\tMl:" << Ml.transpose() << "\tMr:" << Mr.transpose();

    vel = nudgePGain*( Ma.cross(Vector3d(0,0,1)) );

//    std::cout << "V0:" << vel.transpose();

    // Prevent pushing past ankle roll limit
    Eigen::Vector3d fhaty;
    fhaty(0) = -sin(hubo.getJointAngleState(LHY));
    fhaty(1) =  cos(hubo.getJointAngleState(LHY));
    fhaty(2) =  0;

    if( hubo.getJointAngle(LAR) >= hubo.getJointAngleMax(LAR)-craneAnkleRollThresh
            && vel.dot(fhaty) > 0 )
        vel = vel - vel.dot(fhaty)*fhaty;
    else if( hubo.getJointAngle(LAR) <= hubo.getJointAngleMin(LAR)+craneAnkleRollThresh
            && vel.dot(fhaty) < 0 )
        vel = vel - vel.dot(fhaty)*fhaty;

    fhaty(0) = -sin(hubo.getJointAngleState(RHY));
    fhaty(1) =  cos(hubo.getJointAngleState(RHY));

    if( hubo.getJointAngle(RAR) >= hubo.getJointAngleMax(RAR)-craneAnkleRollThresh
            && vel.dot(fhaty) > 0 )
        vel = vel - vel.dot(fhaty)*fhaty;
    else if( hubo.getJointAngle(RAR) <= hubo.getJointAngleMin(RAR)+craneAnkleRollThresh
            && vel.dot(fhaty) < 0 )
        vel = vel - vel.dot(fhaty)*fhaty;

//    std::cout << "\tV1:" << vel.transpose();

    state.verr += vel*dt;
    vel += nudgeIGain*state.verr;
    vel -= nudgeDGain*(vel-state.vprev); 
//    verr += (vel - nudgeIGain*verr)*dt;

    std::cout << "\tV2:" << vel.transpose();


    hubo.hipVelocityIK( lqdot, vel, LEFT );
    hubo.hipVelocityIK( rqdot, vel, RIGHT );


    double refAngleX=0, refAngleY=0;
    // TODO: Calculate refAngleX and refAngleY from the desired IMU orientation
    
    state.rarerr += craneAngleMultiplier*(rollAngleGain*hubo.getAngleX()-refAngleX)
                + craneCompMultiplier*(compRollGain*hubo.getRightFootMx()-elem.torque[RIGHT][0]);
    rqdot(AR) += state.rarerr;
    state.raperr += craneAngleMultiplier*(pitchAngleGain*hubo.getAngleY()-refAngleY)
                + craneCompMultiplier*(compPitchGain*hubo.getRightFootMy()-elem.torque[RIGHT][1]);
    rqdot(AP) += state.raperr;

    state.larerr += craneAngleMultiplier*(rollAngleGain*hubo.getAngleX()-refAngleX)
                + craneCompMultiplier*(compRollGain*hubo.getLeftFootMx()-elem.torque[LEFT][0]);
    lqdot(AR) += state.larerr;
    state.laperr += craneAngleMultiplier*(pitchAngleGain*hubo.getAngleY()-refAngleY)
                + craneCompMultiplier*(compPitchGain*hubo.getLeftFootMy()-elem.torque[LEFT][1]);
    lqdot(AP) += state.laperr;

    std::cout << "\trqdot:" << rqdot.transpose();
}


void singleLegNudge( Hubo_Control &hubo, const zmp_traj_element_t &elem, int side, 
                           Vector6d lqdot, Vector6d rqdot, foot_state_t &state, double dt )
//                        Eigen::Vector3d &vprev,
//                        Eigen::Vector3d &verr, double dt )
{
    Eigen::Vector3d vel; vel.setZero();
    rqdot.setZero();
    lqdot.setZero();

    double refAngleX=0, refAngleY=0;
    // TODO: Calculate refAngleX and refAngleY from the desired IMU orientation

    if( side == RIGHT )
    {
        vel(0) =  craneShiftGainX*( hubo.getRightFootMy() - elem.torque[RIGHT][1] );
        vel(1) = -craneShiftGainY*( hubo.getRightFootMx() - elem.torque[RIGHT][0] );

        state.verr += vel*dt;
        vel += nudgeIGain*state.verr;
        vel -= nudgeDGain*(vel-state.vprev); 

        hubo.hipVelocityIK( rqdot, vel, RIGHT );
        
        state.rarerr += craneAngleMultiplier*(rollAngleGain*hubo.getAngleX()-refAngleX)
                    + craneCompMultiplier*(compRollGain*hubo.getRightFootMx()-elem.torque[RIGHT][0]);
        rqdot(AR) += state.rarerr;
        state.raperr += craneAngleMultiplier*(pitchAngleGain*hubo.getAngleY()-refAngleY)
                    + craneCompMultiplier*(compPitchGain*hubo.getRightFootMy()-elem.torque[RIGHT][1]);
        rqdot(AP) += state.raperr;

        if(    hubo.getJointAngle(RAR) <= hubo.getJointAngleMin(RAR)+craneAnkleRollThresh
            || (hubo.getJointAngleState(RHR) + hubo.getJointAngleState(RAR) < 0.0
                && hubo.getJointAngleState(RAR) < 0 && hubo.getJointAngleState(RHR) > 0) )
        {
            rqdot(AR) = 0.0;
            rqdot(HR) = craneHipRollGain/craneShiftGainY*
                            (vel(1)*cos(hubo.getJointAngleState(RHY))
                            -vel(0)*sin(hubo.getJointAngleState(RHY)));
            lqdot(HR) +=  rqdot(HR)*cos(hubo.getJointAngleState(LHY)-hubo.getJointAngleState(RHY));
            lqdot(HP) += -rqdot(HR)*sin(hubo.getJointAngleState(LHY)-hubo.getJointAngleState(RHY));
        }

        if( elem.angles[RHR]+rqdot(HR) > elem.angles[LHR]+lqdot(HR) )
            lqdot(HR) += rqdot(HR); 
    }
    else
    {
        vel(0) =  craneShiftGainX*( hubo.getLeftFootMy() - elem.torque[LEFT][1] );
        vel(1) = -craneShiftGainY*( hubo.getLeftFootMx() - elem.torque[LEFT][0] );
        
        state.verr += vel*dt;
        vel += nudgeIGain*state.verr;
        vel -= nudgeDGain*(vel-state.vprev); 

        hubo.hipVelocityIK( lqdot, vel, LEFT );

        state.larerr += craneAngleMultiplier*(rollAngleGain*hubo.getAngleX()-refAngleX)
                    + craneCompMultiplier*(compRollGain*hubo.getLeftFootMx()-elem.torque[LEFT][0]);
        lqdot(AR) += state.larerr;
        state.laperr += craneAngleMultiplier*(pitchAngleGain*hubo.getAngleY()-refAngleY)
                    + craneCompMultiplier*(compPitchGain*hubo.getLeftFootMy()-elem.torque[LEFT][1]);
        lqdot(AP) += state.laperr;


        if(    hubo.getJointAngle(LAR) >= hubo.getJointAngleMax(LAR)-craneAnkleRollThresh
            || (hubo.getJointAngleState(LHR) + hubo.getJointAngleState(LAR) > 0.0
                && hubo.getJointAngleState(LAR) > 0 && hubo.getJointAngleState(LHR) < 0) )
        {
            lqdot(AR) = 0.0;
            lqdot(HR) = craneHipRollGain/craneShiftGainY*
                            (vel(1)*cos(hubo.getJointAngleState(LHY))
                            -vel(0)*sin(hubo.getJointAngleState(LHY)));
            rqdot(HR) +=  lqdot(HR)*cos(hubo.getJointAngleState(RHY)-hubo.getJointAngleState(LHY));
            rqdot(HP) += -lqdot(HR)*sin(hubo.getJointAngleState(RHY)-hubo.getJointAngleState(LHY));
        }

        if( elem.angles[RHR]+rqdot(HR) > elem.angles[LHR]+lqdot(HR) )
            rqdot(HR) += lqdot(HR); 
    }
    

}


void nudgeRefs( Hubo_Control &hubo, zmp_traj_element_t &elem, //Eigen::Vector3d &vprev,
                            foot_state_t &state, double dt )
//                Eigen::Vector3d &verr, double dt )
{
    Vector6d lqdot, rqdot; lqdot.setZero(); rqdot.setZero();

    if( elem.stance == DOUBLE_LEFT || elem.stance == DOUBLE_RIGHT )
        dualLegNudge( hubo, elem, lqdot, rqdot, state, dt );
    else if( elem.stance == SINGLE_LEFT )
        singleLegNudge( hubo, elem, LEFT, lqdot, rqdot, state, dt );
    else if( elem.stance == SINGLE_RIGHT )
        singleLegNudge( hubo, elem, RIGHT, lqdot, rqdot, state, dt );

    for(int i=0; i < LEG_JOINT_COUNT; i++)
    {
        elem.angles[leftlegjoints[i]] += lqdot(i)*dt;
        elem.angles[rightlegjoints[i]] += rqdot(i)*dt;
    }
}


int main(int argc, char **argv)
{
    Hubo_Control hubo;
    foot_state_t state;
    memset( &state, 0, sizeof(foot_state_t) );
//    Eigen::Vector3d verr; verr.setZero();
//    Eigen::Vector3d vprev; vprev.setZero();

    ach_status_t r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    fprintf( stderr, "%s (%d)\n", ach_result_to_string(r), (int)r );
    
    
    size_t fs;
    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );
    ach_get( &zmp_chan, &trajectory, sizeof(trajectory), &fs, NULL, ACH_O_LAST );

    fprintf(stderr, "Count: %d\n", trajectory.count);
    for(int i=0; i<trajectory.count; i++)
        fprintf(stdout, "%d: RHR %f\n", i, trajectory.traj[i].angles[RHR] );


    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.setJointAngle( i, trajectory.traj[0].angles[i] );
        hubo.setJointNominalSpeed( i, 0.4 );
        hubo.setJointNominalAcceleration( i, 0.4 );
    }

    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.setJointAngle( RSR, trajectory.traj[0].angles[RSR] + hubo.getJointAngleMax(RSR) );
    hubo.setJointAngle( LSR, trajectory.traj[0].angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.sendControls();

    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
//    while( time - stime < 7 )
    while(true)
    {
        hubo.update(true);
        dt = hubo.getTime() - time;
        time = hubo.getTime();

        zmp_traj_element_t init = trajectory.traj[0];
        nudgeRefs( hubo, init, state, dt ); //vprev, verr, dt );

        printf("B:%f\tA:%f\tD:%f\n", trajectory.traj[0].angles[RHP], init.angles[RHP],
                                        init.angles[RHP]-trajectory.traj[0].angles[RHP] );

        for(int i=0; i<HUBO_JOINT_COUNT; i++)
            hubo.setJointAngle( i, init.angles[i] );
        hubo.setJointAngle( RSR, init.angles[RSR] + hubo.getJointAngleMax(RSR) );
        hubo.setJointAngle( LSR, init.angles[LSR] + hubo.getJointAngleMin(LSR) );

        hubo.setJointAngleMin( LHR, init.angles[RHR] );
        hubo.setJointAngleMax( RHR, init.angles[LHR] );
        hubo.sendControls();
    }

    printf("Time elapsed\n");






/*
    fprintf(stdout, "%d\n", trajectory.count);
    for(int t=1; t<trajectory.count-1; t++)
    {
        hubo.update(true);
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            hubo.setJointAngle( i, trajectory.traj[t].angles[i] );
            hubo.setJointNominalSpeed( i,
                   (trajectory.traj[t].angles[i]-trajectory.traj[t-1].angles[i])*TRAJ_FREQ_HZ );
            double accel = TRAJ_FREQ_HZ*TRAJ_FREQ_HZ*(
                                trajectory.traj[t-1].angles[i]
                            - 2*trajectory.traj[t].angles[i]
                            +   trajectory.traj[t+1].angles[i] );
            if( i==RHR ) fprintf( stdout, "RHR Accel: %f\n", accel );
            hubo.setJointNominalAcceleration( i, 10*accel );
        }

        hubo.setJointAngle( RSR, trajectory.traj[t].angles[RSR] + hubo.getJointAngleMax(RSR) );
        hubo.setJointAngle( LSR, trajectory.traj[t].angles[LSR] + hubo.getJointAngleMin(LSR) );

        hubo.setJointAngleMin( LHR, trajectory.traj[t].angles[RHR] );
        hubo.setJointAngleMax( RHR, trajectory.traj[t].angles[LHR] );
        hubo.sendControls();
    }
*/











/*
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
//            placeSwingFoot( LEFT, placeFoot, hubo, dt );


//            liftLeg( RIGHT, 0.15, hubo, dt );
            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else if(!go)
                go = placeSwingFoot( RIGHT, placeFoot, hubo, dt );
            else
                craneStance( LEFT, hubo, dt );

//            shiftToSide( LEFT, hubo, trans, dt );

//          *** Shifting weight endurance test
            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else
                ready = !shiftToSide( RIGHT, hubo, trans, dt );

//          *** Standard shift & crane
            if( !ready )
                ready = shiftToSide( LEFT, hubo, trans, dt );
            else if(!go)
                go = placeSwingFoot( RIGHT, placeFoot, hubo, dt );
            else
                craneStance( LEFT, hubo, dt );

        }
    }

*/


}
