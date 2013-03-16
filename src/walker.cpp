#include "walker.h"

#define HAVE_HUBO_ACH
#include <hubo-zmp.h>

#include "HuboKin.h"


ach_channel_t zmp_chan;



/*
const double nudgePGain = 0.04;
const double nudgeIGain = 0.2;
const double nudgeDGain = 0.0;
*/


// gain for ankle flat
const double k_tau = 0.0015; // Grey likes this value

// gain for moment reducing
const double k_pos = 0.0015; // Crane stance resistance to torque

// gain for IMU rotation
const double k_theta = 0; //0.5*M_PI/180.0; // IMU feedback gain

const double weight_thresh_N = -1e5; // TESTING k_tau 30; // weight to turn on/off integrators
const double nudge_max_norm = 0.05; // m
const double spin_max_angle = 10 * M_PI/180;
const double comp_max_angle = 30 * M_PI/180;

//const double compPitchGain = 0.0015;
//const double shiftCompMultiplier = 1.0;
//const double craneCompMultiplier = 1.0;


// TODO: resurrect this later when roll ankle limits give us trouble
//
//const double craneHipPitchGain = 0.0*M_PI/180.0; // Not used
//const double craneHipRollGain  = 0.05*M_PI/180.0; // Used when ankle roll reaches joint limits
//const double craneAnkleRollThresh = 2.0*M_PI/180.0;

//const double craneHipSafetyMargin = 2.5*M_PI/180.0; // Buffer to prevent leg collisions during crane stance
//const double craneHipSafetyGain = 1.5;

//const double swingVelLimit = 0.05;

//const double rollAngleGain = 0.3*M_PI/180.0; // IMU feedback gain
//const double craneAngleMultiplier = 1.0;

//const double shiftGain = 0.0002; // Shift-to-side gain
//const double shiftBalanceGain = 0.0015; // Gain to maintain balance while shifting
//const double craneTransitionPercent = 98.0; // Percent of weight on swing foot before entering crane stance

//const double crouchGain = 0.30;  // Leg-lifting gain

//const double pcAccel = 0.4; // Nominal acceleration used for position control
//const double vcAccel = 1.5; // Nominal acceleration used for velocity control

//const double calKneeGain = 1.0;
//const double calWeightGain = 0.001; // rad/sec per N


void getLegNudge( Hubo_Control &hubo, Vector6d q, 
		  zmp_traj_element_t elem, nudge_state_t &state, 
		  int side, double dt )
{
    Eigen::Vector3d vel; vel.setZero();

    double refAngleX=0, refAngleY=0;
    // TODO: Calculate desired IMU angle

    Eigen::Matrix3d R = Eigen::AngleAxisd(q(HY), Vector3d::UnitZ()).toRotationMatrix();

    if( side==RIGHT )
    {
        vel(0) =  k_pos*( hubo.getRightFootMy() - elem.torque[RIGHT][1] );
        vel(1) = -k_pos*( hubo.getRightFootMx() - elem.torque[RIGHT][0] );


        state.nudge += R*vel*dt;

        state.spin(0) += dt*k_theta*(hubo.getAngleX()-refAngleX);//-state.imu_offset(0));
        state.spin(1) += dt*k_theta*(hubo.getAngleY()-refAngleY);//-state.imu_offset(0));

        state.rarerr += dt*k_tau*( hubo.getRightFootMx()-elem.torque[RIGHT][0] );
        state.raperr += dt*k_tau*( hubo.getRightFootMy()-elem.torque[RIGHT][1] );
    }
    else
    {
        vel(0) =  k_pos*( hubo.getLeftFootMy() - elem.torque[LEFT][1] );
        vel(1) = -k_pos*( hubo.getLeftFootMx() - elem.torque[LEFT][0] );

        state.nudge += R*vel*dt;

        state.spin(0) += dt*k_theta*(hubo.getAngleX()-refAngleX);//-state.imu_offset(0));
        state.spin(1) += dt*k_theta*(hubo.getAngleY()-refAngleY);//-state.imu_offset(0));

        state.larerr += dt*k_tau*( hubo.getLeftFootMx()-elem.torque[LEFT][0] );
        state.laperr += dt*k_tau*( hubo.getLeftFootMy()-elem.torque[LEFT][1] );
    }



}

static inline void clamp(double& x, double cap) {
  x = std::max(-cap, std::min(x, cap));
}

// input is a hubo_control
// qr & ql are planned joint angles - used for 
// state holds the nudge integrators
// elem is our zmp_traj_element_t
// dt is likely to be 1.0/TRAJ_FREQ_HZ
void integrateNudgeIK( Hubo_Control &hubo, Vector6d &qr, Vector6d& ql,
		       nudge_state_t& state, zmp_traj_element_t elem, double dt,
		       HK::HuboKin& hkin) {

  // now truncate stuff
  if (state.nudge.norm() > nudge_max_norm) {
    state.nudge *= nudge_max_norm / state.nudge.norm();
  }

  clamp(state.spin[0], spin_max_angle);
  clamp(state.spin[1], spin_max_angle);
  clamp(state.larerr,  comp_max_angle);
  clamp(state.laperr,  comp_max_angle);
  clamp(state.rarerr,  comp_max_angle);
  clamp(state.raperr,  comp_max_angle);


  Vector6d* qboth[2] = { &qr, &ql };

  for (int side=0; side<2; ++side) {


    // Matt being paranoid: don't run IK code if no nudging to be done.
    if (state.nudge.squaredNorm() || state.spin.squaredNorm()) {

      Vector6d& qs = *(qboth[side]);


      Eigen::Isometry3d FK;

      // right comes first in HuboKin cause it was written by right handed people?
      hkin.legFK(FK, qs, side);

      // FK maps foot frame to body
      // 

      const Vector6d qs_prev = qs;

      const Eigen::Vector3d p = -state.nudge;
    
      const Eigen::Quaterniond R = 
	Eigen::AngleAxisd(state.spin[0], Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxisd(state.spin[1], Eigen::Vector3d::UnitY());

      std::cout << "nudge for leg " << side << " is " << p.transpose() << "\n";
      std::cout << "spin for leg " << side << " is " << state.spin.transpose() << "\n";

      Eigen::Isometry3d Tdelta;
      Tdelta.setIdentity();
      Tdelta.rotate(R);
      Tdelta.pretranslate(p);

      std::cout << "Tdelta for leg " << side << "=\n" << Tdelta.matrix() << "\n";
      Eigen::Isometry3d desired = Tdelta * FK;

      //Eigen::Isometry3d desired = FK;

      hkin.legIK(qs, desired, qs_prev, side);

      std::cout << "Diff for leg " << side << " is " << (qs_prev - qs).transpose() << "\n";


    }

  }

  qr(AR) += state.rarerr;
  qr(AP) += state.raperr;

  ql(AR) += state.larerr;
  ql(AP) += state.laperr;


}

/*
// input is a hubo_control
// qr & ql are planned joint angles - used for 
// state holds the nudge integrators
// elem is our zmp_traj_element_t
// dt is likely to be 1.0/TRAJ_FREQ_HZ
void integrateNudge( Hubo_Control &hubo, Vector6d &qr, Vector6d& ql,
                        nudge_state_t state, zmp_traj_element_t elem, double dt )
{
    Vector6d dt_qr, dt_ql;
    int num = 20;
    double ddt = dt/num;
    

    for(int i=0; i<num; i++)
    {
        hubo.hipVelocityIK( dt_qr, state.nudge, state.spin, qr );
        hubo.hipVelocityIK( dt_ql, state.nudge, state.spin, ql );

        if( elem.stance == DOUBLE_LEFT || DOUBLE_RIGHT )
        {
            if( qr(AR)+dt_qr(AR)*ddt <= hubo.getJointAngleMin(RAR)
                || ( qr(HR)+qr(AR)<0.0 && qr(AR)<0 && qr(HR)>0 ) )
            {
                Eigen::Vector3d ghat( -sin(qr(HY)), cos(qr(HY)), 0 );
                Eigen::Vector3d altNudge = state.nudge - (state.nudge.dot(ghat))*ghat;

                hubo.hipVelocityIK( dt_qr, altNudge, state.spin, qr );
                hubo.hipVelocityIK( dt_ql, altNudge, state.spin, ql );
            }
            
            if( ql(AR)+dt_ql(AR)*ddt >= hubo.getJointAngleMax(LAR)
                || ( ql(HR)+ql(AR)>0.0 && ql(AR)>0 && ql(HR)<0 ) ) 
            {
                Eigen::Vector3d ghat( -sin(ql(HY)), cos(ql(HY)), 0 );
                Eigen::Vector3d altNudge = state.nudge - (state.nudge.dot(ghat))*ghat;

                hubo.hipVelocityIK( dt_qr, altNudge, state.spin, qr );
                hubo.hipVelocityIK( dt_ql, altNudge, state.spin, ql );
            }

            qr += dt_qr*dt;
            ql += dt_ql*dt;

        }
        else if( elem.stance == SINGLE_RIGHT )
        {
            if( qr(AR)+dt_qr(AR)*ddt <= hubo.getJointAngleMin(RAR)
                || ( qr(HR)+qr(AR)<0.0 && qr(AR)<0 && qr(HR)>0 ) )
            {
                Eigen::Vector3d ghat( -sin(qr(HY)), cos(qr(HY)), 0 );
                Eigen::Vector3d altNudge = state.nudge - (state.nudge.dot(ghat))*ghat;
                Eigen::Vector3d altSpin  = state.spin + craneHipRollGain/k_pos*
                                            ( (state.nudge.dot(ghat))*(ghat.cross(Eigen::Vector3d(0,0,1))) );
                state.imu_offset += (altSpin - state.spin)*ddt; 
                
                hubo.hipVelocityIK( dt_qr, altNudge, altSpin, qr );
                hubo.hipVelocityIK( dt_ql, altNudge, altSpin, ql );
            }

            qr += dt_qr*dt;
            ql += dt_ql*dt;

        }
        else if( elem.stance == SINGLE_LEFT )
        {
            if( ql(AR)+dt_ql(AR)*ddt >= hubo.getJointAngleMax(LAR)
                || ( ql(HR)+ql(AR)>0.0 && ql(AR)>0 && ql(HR)<0 ) )
            {
                Eigen::Vector3d ghat( -sin(qr(HY)), cos(qr(HY)), 0 );
                Eigen::Vector3d altNudge = state.nudge - (state.nudge.dot(ghat))*ghat;
                Eigen::Vector3d altSpin  = state.spin + craneHipRollGain/k_pos*
                                            ( (state.nudge.dot(ghat))*(ghat.cross(Eigen::Vector3d(0,0,1))) ); 
                state.imu_offset += (altSpin - state.spin)*ddt; 
                
                hubo.hipVelocityIK( dt_qr, altNudge, altSpin, qr );
                hubo.hipVelocityIK( dt_ql, altNudge, altSpin, ql );
            }

            qr += dt_qr*dt;
            ql += dt_ql*dt;

        }
        
    }

    qr(AR) += state.rarerr*ddt;
    qr(AP) += state.raperr*ddt;
    ql(AR) += state.larerr*ddt;
    ql(AP) += state.laperr*ddt;


}
*/

void nudgeRefs( Hubo_Control &hubo, zmp_traj_element_t &elem, //Eigen::Vector3d &vprev,
		nudge_state_t &state, double dt,
		HK::HuboKin& hkin)
{

  Vector6d qr, ql;
  qr(HY) = elem.angles[RHY];
  qr(HR) = elem.angles[RHR];
  qr(HP) = elem.angles[RHP];
  qr(KN) = elem.angles[RKN];
  qr(AP) = elem.angles[RAP];
  qr(AR) = elem.angles[RAR];

  ql(HY) = elem.angles[LHY];
  ql(HR) = elem.angles[LHR];
  ql(HP) = elem.angles[LHP];
  ql(KN) = elem.angles[LKN];
  ql(AP) = elem.angles[LAP];
  ql(AR) = elem.angles[LAR];

  // TODO: smoothly phase in the integrator for each foot
  if( hubo.getRightFootFz()+hubo.getLeftFootFz() > weight_thresh_N) {
        
      nudge_state_t lstate=state, rstate=state;

      getLegNudge( hubo, qr, elem, rstate, RIGHT, dt );
      getLegNudge( hubo, ql, elem, lstate, LEFT,  dt );
        
      state.larerr = lstate.larerr;
      state.rarerr = rstate.rarerr;
      state.laperr = lstate.laperr;
      state.raperr = rstate.raperr;

      if( elem.forces[RIGHT][2]+elem.forces[LEFT][2] == 0 ) {

	elem.forces[RIGHT][2] = 1;
	elem.forces[LEFT][2]  = 1;
	fprintf(stderr, "Warning: predicted forces both 0!");
      }

      state.nudge = (elem.forces[RIGHT][2]*rstate.nudge + elem.forces[LEFT][2]*lstate.nudge)/
	(elem.forces[RIGHT][2]+elem.forces[LEFT][2]);
      state.spin  = (elem.forces[RIGHT][2]*rstate.spin  + elem.forces[LEFT][2]*lstate.spin )/
	(elem.forces[RIGHT][2]+elem.forces[LEFT][2]);


  }


  std::cout << "Nudge: " << state.nudge.transpose() << "\tSpin: " << state.spin.transpose();
  integrateNudgeIK( hubo, qr, ql, state, elem, dt, hkin );

        
  elem.angles[RHY] = qr(HY);
  elem.angles[RHR] = qr(HR);
  elem.angles[RHP] = qr(HP);
  elem.angles[RKN] = qr(KN);
  elem.angles[RAP] = qr(AP);
  elem.angles[RAR] = qr(AR);

  elem.angles[LHY] = ql(HY);
  elem.angles[LHR] = ql(HR);
  elem.angles[LHP] = ql(HP);
  elem.angles[LKN] = ql(KN);
  elem.angles[LAP] = ql(AP);
  elem.angles[LAR] = ql(AR);

}


int main(int argc, char **argv)
{
    Hubo_Control hubo;

    HK::HuboKin hkin;

    hkin.kc.leg_l1 = 0; // eliminate neck -> waist Z distance
    hkin.kc.leg_l3 = 0; // eliminate waist -> hip Z distance
    hkin.kc.leg_l6 = 0; // eliminate ankle -> foot Z distance

    nudge_state_t state;
    memset( &state, 0, sizeof(nudge_state_t) );

    ach_status_t r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    fprintf( stderr, "%s (%d)\n", ach_result_to_string(r), (int)r );
    
    
    size_t fs;
    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );
    ach_get( &zmp_chan, &trajectory, sizeof(trajectory), &fs, NULL, ACH_O_LAST );

    fprintf(stderr, "Count: %d\n", (int)trajectory.count);
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
        init.forces[0][2] = 200; init.forces[1][2] = 200;

        nudgeRefs( hubo, init, state, dt, hkin ); //vprev, verr, dt );

        std::cout << hubo.getRightFootMy() + hubo.getLeftFootMy() << std::endl;

//        printf("B:%f\tA:%f\tD:%f\n", trajectory.traj[0].angles[RHP], init.angles[RHP],
//                                        init.angles[RHP]-trajectory.traj[0].angles[RHP] );

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










}
