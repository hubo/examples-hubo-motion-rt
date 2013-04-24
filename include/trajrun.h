#ifndef TRAJRUN_H
#define TRAJRUN_H

#include <hubo.h>

#define HUBO_TRAJ_MAX_TIME 10 //seconds
#define HUBO_TRAJ_FREQ 200 //Hertz

#define TRAJ_MAX_SIZE HUBO_TRAJ_MAX_TIME*HUBO_TRAJ_FREQ

#define HUBO_TRAJ_CHAN "hubo-traj"
#define HUBO_TRAJ_STATE_CHAN "hubo-traj-state"

typedef struct hubo_joint_traj {

    double position[TRAJ_MAX_SIZE];
    double velocity[TRAJ_MAX_SIZE];
    double acceleration[TRAJ_MAX_SIZE];

} __attribute__((packed)) hubo_joint_traj_t;

typedef struct hubo_traj {

    hubo_joint_traj_t joint[HUBO_JOINT_COUNT];    
    double time[TRAJ_MAX_SIZE];

    double endTime;
    int trajID;

} __attribute__((packed)) hubo_traj_t;


typedef enum {
    
    TRAJ_RUNNING,
    TRAJ_COMPLETE

} traj_status_t;

typedef struct hubo_traj_output {

    traj_status_t status;
    int trajID;
    
    double error[HUBO_JOINT_COUNT];

} __attribute__((packed)) hubo_traj_output_t;


#endif //TRAJRUN_H
