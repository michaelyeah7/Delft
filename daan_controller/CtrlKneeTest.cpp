#include "CtrlKneeTest.h"
#include <iostream>
#include <fstream>
using namespace std;


CKneeTestController	gKneeTestController;
CKneeTest_StEnd		gKneeTest_StEnd;
CKneeTest_StKnee	gKneeTest_StKnee;


float 	knee_q_start_test[2];


// #ifndef max
// #define max(a, b)  (((a) > (b)) ? (a) : (b))
// #endif

//****************************************************************/
// The standing controller
//****************************************************************/

void CKneeTestController::Init()
{
	// disable all controllers
	joints.hipx.Disable();
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].hipy.Disable();
		joints.legs[iLeg].knee.Disable();
		joints.legs[iLeg].ankley.Disable();
		
		joints.interleg.Disable();
		joints.upperbody.Disable();
	}
	
	s.powered = 0;//FLAME_ALL_MOTORS;
	
	// Goto Hipx movement
	Transition(&gKneeTest_StKnee);
}

void CKneeTest_StKnee::Init()
{
    // ofstream myfile;
    // myfile.open ("example.txt");
    // myfile << "Writing this to a file.\n";
    // myfile.close();
	logprintf("Start Knee motion\n");
		
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			// store angle knee at start motion
			knee_q_start_test[iLeg] = s.legs[iLeg].kneemot.q;
			
			joints.legs[iLeg].knee.mMode = SEA_MODE_MOTANGLE;
			joints.legs[iLeg].knee.motangCtrl.kp = 25;
		}
		
		s.powered = FLAME_HIPX_MOTOR | FLAME_LHIPY_MOTOR | FLAME_RHIPY_MOTOR | FLAME_LKNEE_MOTOR | FLAME_RKNEE_MOTOR;
}

void CKneeTest_StKnee::Update()
{
	float knee_maxrefangle = 0.6;
    ofstream myfile;
    myfile.open ("kneedebug.txt");
	logprintf("TimeElapsed %f\n",Controller()->TimeElapsed());
    // logprintf("knee_q_start_test[0]=%f\n",knee_q_start_test[0]);
    // logprintf("knee_q_start_test[1]=%f\n",knee_q_start_test[1]);
	if (Controller()->TimeElapsed() <= 1.0)
	{
        logprintf("Moving to Max angle\n");
		joints.l().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start_test[0], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
		joints.r().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start_test[1], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
	}
		else
		{
			if (Controller()->TimeElapsed() <= 2.0)
			{
                logprintf("Moving back to zero angle\n");
				joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
			}
		}
	
	joints.Update(&s);
    logprintf("left_q=%f,right_q=%f,left_qmot_ref=%f,left_qmot=%f,right_qmot_ref=%f,right_qmot=%f,left_torque=%f,right_torque=%f",
	s.legs[0].knee.q, s.legs[1].knee.q,joints.l().knee.qmot.ref,s.legs[0].kneemot.q,joints.r().knee.qmot.ref,s.legs[1].kneemot.q,s.legs[0].knee.tau,s.legs[1].knee.tau);
    // myfile << s.legs[0].knee.q <<',' << s.legs[1].knee.q <<',' << joints.l().knee.qmot.ref <<',' << s.legs[0].kneemot.q <<',' << joints.r().knee.qmot.ref <<',' << s.legs[1].kneemot.q <<',' << s.legs[0].knee.tau <<',' << s.legs[1].knee.tau; 
	myfile << s.legs[0].knee.q;
    logprintf("new timestep data written");
    myfile.close();
	if (Controller()->TimeElapsed() >= 2.0){
        logprintf("KneeTest completed\n");
        // myfile.close();
		Controller()->Transition(&gKneeTest_StEnd);
    }
}
