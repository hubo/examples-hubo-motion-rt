#include <Hubo_Control.h>
#include <vector>
#include <iostream>
#include <fstream>
// Note that "std::vector" is a dynamic array class in C++ (not available in C)
// This means you can use std::vector to make a variable-sized array of ArmVector
// Even though std::vector and ArmVector have similar names, they are not directly related to each other
using namespace std;

double* getArf(char* s);
void printDoubleArray (double array[]);
void gotoFirstPosition(double referenceData[], Hubo_Control &hubo);
void gotoNewPosition(double referenceData[], double bufferedData[], int resample_ratio, Hubo_Control &hubo, FILE * resultFile);
double* interpolate_linear (double referenceData[], double bufferedData[], double multiplier);
#define number_of_joints 40 //because the file has 40 elements
bool checkTrajectory (double nextPosition[], double currentPosition[]);
 
double* getArg(char* s) {

	double *r= new double[number_of_joints];
	sscanf(s, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        &r[0],
        &r[1],
        &r[2],
        &r[3],
        &r[4],
        &r[5],
        &r[6],
        &r[7],
        &r[8],
        &r[9],
        &r[10],
        &r[11],
        &r[12],
        &r[13],
        &r[14],
        &r[15],
        &r[16],
        &r[17],
        &r[18],
        &r[19],
        &r[20],
        &r[21],
        &r[22],
        &r[23],
        &r[24],
        &r[25],
        &r[26],
        &r[27],
        &r[28],
        &r[29],
        &r[30],
        &r[31],
        &r[32],
        &r[33],
        &r[34],
        &r[35],
        &r[36],
        &r[37],
        &r[38],
        &r[39]);

        return r;
}

void printDoubleArray (double array[]){
	for (int i=0; i<number_of_joints; i++){
		cout<<array[i];
		cout<<", ";
	}
	cout<<"\n-----------------------------------------------------------\n";	
}

void gotoFirstPosition(double referenceData[], Hubo_Control &hubo){
    double tol = 0.075; // This will be the allowed tolerance before moving to the next point
    ArmVector  left_arm_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  right_arm_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  left_leg_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  right_leg_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 

    left_arm_angles<< referenceData[LSP], referenceData[LSR], referenceData[LSY], referenceData[LEB], referenceData[LWY], referenceData[LWP], referenceData[LWR], 0,0,0;
    right_arm_angles<< referenceData[RSP], referenceData[RSR], referenceData[RSY], referenceData[REB], referenceData[RWY], referenceData[RWP], referenceData[RWR],0,0,0;
    
    right_leg_angles<< referenceData[RHY], referenceData[RHR], referenceData[RHP], referenceData[RKN], referenceData[RAP], referenceData[RAR],0,0,0,0;
    left_leg_angles<< referenceData[LHY], referenceData[LHR], referenceData[LHP], referenceData[LKN], referenceData[LAP], referenceData[LAR],0,0,0,0;

    bool left_arm_in_limit=false;
    bool right_arm_in_limit=false;
    bool left_leg_in_limit=false;
    bool right_leg_in_limit=false;
    ArmVector currentPosition;  // This ArmVector will be used to track our current angle configuration
 
    while( left_arm_in_limit && right_arm_in_limit && left_leg_in_limit && right_leg_in_limit )
    {
        hubo.update(true); // This grabs the latest state information
	
	
        hubo.getLeftArmAngleStates( currentPosition ); // This will fill in the values of "currentPosition" by passing it in by reference
        // If you are unfamiliar with "passing by reference", let me know and I can explain the concept. It's a feature of C++ but not C
        if( ( currentPosition-left_arm_angles ).norm() < tol ) // The class function .norm() is a feature of the Eigen libraries. Eigen has many other extremely useful functions like this.
        {
        	left_arm_in_limit=true;
	}

        hubo.getRightArmAngleStates( currentPosition ); // This will fill in the values of "currentPosition" by passing it in by reference
        if( ( currentPosition-right_arm_angles).norm() < tol ) // The class function .norm() is a feature of the Eigen libraries. Eigen has many other extremely useful functions like this.
        {
        	right_arm_in_limit=true;
	}

        hubo.getLeftLegAngleStates( currentPosition ); // This will fill in the values of "currentPosition" by passing it in by reference
        if( ( currentPosition-left_leg_angles ).norm() < tol ) // The class function .norm() is a feature of the Eigen libraries. Eigen has many other extremely useful functions like this.
        {
        	left_leg_in_limit=true;
	}

        hubo.getRightLegAngleStates( currentPosition ); // This will fill in the values of "currentPosition" by passing it in by reference
        if( ( currentPosition-right_leg_angles ).norm() < tol ) // The class function .norm() is a feature of the Eigen libraries. Eigen has many other extremely useful functions like this.
        {
        	right_leg_in_limit=true;
	}

        hubo.setLeftArmAngles( left_arm_angles); // Notice that the second argument is not passed in, making it default to "false"
        hubo.setRightArmAngles( right_arm_angles); // Notice that the second argument is not passed in, making it default to "false"
        hubo.setLeftLegAngles( left_leg_angles); // Notice that the second argument is not passed in, making it default to "false"
        hubo.setRightLegAngles( right_leg_angles); // Notice that the second argument is not passed in, making it default to "false"
     
        hubo.sendControls(); // This will send off all the latest control commands over ACH
    }

}

void gotoNewPosition(double referenceData[], double bufferedData[], int resample_ratio, Hubo_Control &hubo, FILE * resultFile){
    ArmVector  left_arm_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  right_arm_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  left_leg_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 
    ArmVector  right_leg_angles; // This declares "angles" as a dynamic array of ArmVectors with a starting array length of 5 

    double* interpolatedData= new double[number_of_joints];

    int* joint_array = new int[number_of_joints];
        joint_array[0]=RHY;   
        joint_array[1]=RHR;   
        joint_array[2]=RHP;   
        joint_array[3]=RKN;   
        joint_array[4]=RAP;   
        joint_array[5]=RAR;   
        joint_array[6]=LHY;   
        joint_array[7]=LHR;   
        joint_array[8]=LHP;   
        joint_array[9]=LKN;   
        joint_array[10]=LAP;   
        joint_array[11]=LAR;   
        joint_array[12]=RSP;   
        joint_array[13]=RSR;   
        joint_array[14]=RSY;   
        joint_array[15]=REB;   
        joint_array[16]=RWY;   
        joint_array[17]=RWR;   
        joint_array[18]=RWP;   
        joint_array[19]=LSP;   
        joint_array[20]=LSR;   
        joint_array[21]=LSY;   
        joint_array[22]=LEB;   
        joint_array[23]=LWY;    
        joint_array[24]=LWR;   
        joint_array[25]=LWP;  
        joint_array[26]=NKY;   
        joint_array[27]=NK1;   
      	joint_array[28]=NK2;   
        joint_array[29]=WST;   
        joint_array[30]=RF1;   
        joint_array[31]=RF2;   
        joint_array[32]=RF3;   
        joint_array[33]=RF4;   
       	joint_array[34]=RF5;   
       	joint_array[35]=LF1;   
       	joint_array[36]=LF2;   
       	joint_array[37]=LF3;   
       	joint_array[38]=LF4;  
       	joint_array[39]=LF5;    
    
     //printf("---------------------------------------\n");

     checkTrajectory(referenceData, bufferedData);
     for (int iterator=1; iterator<=resample_ratio; iterator++){

	    double multiplier = (double)iterator/(double)resample_ratio;
	    interpolatedData = interpolate_linear(referenceData, bufferedData, multiplier); 

	    left_arm_angles<< interpolatedData[LSP], interpolatedData[LSR], interpolatedData[LSY], interpolatedData[LEB], interpolatedData[LWY], interpolatedData[LWP], interpolatedData[LWR],0,0,0;
	    right_arm_angles<< interpolatedData[RSP], interpolatedData[RSR], interpolatedData[RSY], interpolatedData[REB], interpolatedData[RWY], interpolatedData[RWP], interpolatedData[RWR],0,0,0;
    
	    right_leg_angles<< interpolatedData[RHY], interpolatedData[RHR], interpolatedData[RHP], interpolatedData[RKN], interpolatedData[RAP], interpolatedData[RAR],0,0,0,0;
	    left_leg_angles<< interpolatedData[LHY], interpolatedData[LHR], interpolatedData[LHP], interpolatedData[LKN], interpolatedData[LAP], interpolatedData[LAR],0,0,0,0;

	    hubo.update(true);

    	for (int joint=0; joint<number_of_joints; joint++){
		if (joint_array[joint]!=WST){
	 		hubo.passJointAngle(joint_array[joint], interpolatedData[joint]);
			fprintf(resultFile,"%f ",interpolatedData[joint]);
		}
	}
	fprintf(resultFile," \n"); 
	fflush(resultFile);
 	hubo.sendControls(); // This will send off all the latest control commands over ACH
 
    }// end of iterator loop
}

double* interpolate_linear (double referenceData[], double bufferedData[], double multiplier){
	if (multiplier >1){
		multiplier=1;
	}
	double* interpolatedData = new double[number_of_joints];
	for (int joint=0; joint<number_of_joints; joint++){
		interpolatedData[joint]=bufferedData[joint]+(referenceData[joint]-bufferedData[joint])*multiplier;
	}
	return interpolatedData;
}

bool checkTrajectory (double nextPosition[], double currentPosition[]){
        bool is_correct=true;
        double threshold =0.03;
        for (int joint=0; joint<number_of_joints; joint++){
                if (abs(nextPosition[joint]-currentPosition[joint])>threshold){
                        is_correct=false;
                        printf("\n too much jump in the joint %d -- from %f to %f ", joint, currentPosition[joint], nextPosition[joint]);
                }
        }
        return is_correct;
}



int main(int argc, char *argv[]) {
    	printf("starting the follow trajectory \n");
	Hubo_Control hubo;
	
	printf("after \n");
	fflush(stdout);
	char str[1000];
        FILE *fp;               // file pointer
	char* filename ="./src/trajectory-file.traj";
        bool first_line=true;
	int frequency=200;
	int input_file_frequency=25;
	int resample_ratio=frequency/input_file_frequency;
	int line_counter=0;
/*
	if (argc>0){
	//	filename=argv[0];
		printf("file is  %c \n",argv[0]);
	}
	if (argc>1){
	//	input_file_frequency=atoi(argv[1]);
		printf(" input freq is  %d  \n",atoi(argv[1])); 
	}
*/
	fp = fopen(filename,"r");
        if(!fp) {
                printf("No Trajectory File!!!\n");
                return 1;  // exit if not file
        }
  	printf ("starting the follow \n");	
	double* referenceData = new double[number_of_joints];
	double* bufferedData  = new double[number_of_joints];
	FILE * resultFile;
	resultFile =fopen("./src/result.traj","w");
        while(fgets(str,sizeof(str),fp) != NULL) {
		line_counter++;
		referenceData=getArg(str);
		//printDoubleArray(referenceData);
		if (first_line==true){
			// goto first position
			gotoFirstPosition(referenceData, hubo);
			bufferedData=referenceData;
			first_line=false;
			printf("first line read and buffered \n");
		}	
		else{
			//normal trajectory following
			//printf("running trajectory on line %d \n", line_counter);
			gotoNewPosition(referenceData, bufferedData, resample_ratio, hubo, resultFile);
			bufferedData=referenceData;
		}
	}
	fclose(resultFile);
	fclose(fp);
}




/*
        &r->ref[RHY],	0
        &r->ref[RHR],	1
        &r->ref[RHP],	2
        &r->ref[RKN],	3
        &r->ref[RAP],	4
        &r->ref[RAR],	5
        &r->ref[LHY],	6
        &r->ref[LHR],	7
        &r->ref[LHP],	8
        &r->ref[LKN],	9
        &r->ref[LAP],	10
        &r->ref[LAR],	11
        &r->ref[RSP],	12
        &r->ref[RSR],	13
        &r->ref[RSY],	14
        &r->ref[REB],	 5
        &r->ref[RWY],	 6
        &r->ref[RWR],	 7
        &r->ref[RWP],	 8
        &r->ref[LSP],	 9
        &r->ref[LSR],	20
        &r->ref[LSY],	21
        &r->ref[LEB],	 2
        &r->ref[LWY],	 3 
        &r->ref[LWR],	 4
        &r->ref[LWP],	 5
        &r->ref[NKY],	 6
        &r->ref[NK1],	 7
        &r->ref[NK2],	 8
        &r->ref[WST],	 9
        &r->ref[RF1],	30
        &r->ref[RF2],	31
        &r->ref[RF3],	 2
        &r->ref[RF4],	 3
        &r->ref[RF5],	 4
        &r->ref[LF1],	 5
        &r->ref[LF2],	 6
        &r->ref[LF3],	 7
        &r->ref[LF4],	 8
        &r->ref[LF5]	 9
*/ 
