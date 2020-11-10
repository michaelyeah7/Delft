#include "CtrlKneeTest.h"

CKneeTestController	gKneeTestController;
CKneeTest_StEnd		gKneeTest_StEnd;
CKneeTest_StKnee	gKneeTest_StKnee;


float 	knee_q_start[2];

#ifndef max
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#endif

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
	logprintf("Start Knee motion\n");
		
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			// store angle knee at start motion
			knee_q_start[iLeg] = s.legs[iLeg].kneemot.q;
			
			joints.legs[iLeg].knee.mMode = SEA_MODE_MOTANGLE;
			joints.legs[iLeg].knee.motangCtrl.kp = 25;
		}
		
		s.powered = FLAME_HIPX_MOTOR | FLAME_LHIPY_MOTOR | FLAME_RHIPY_MOTOR | FLAME_LKNEE_MOTOR | FLAME_RKNEE_MOTOR;
}

void CKneeTest_StKnee::Update()
{
	float knee_maxrefangle = 0.6;
		
	if (Controller()->TimeElapsed() <= 1.0)
	{
		joints.l().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start[0], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
		joints.r().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start[1], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
	}
		else
		{
			if (Controller()->TimeElapsed() <= 2.0)
			{
				joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
			}
		}
	
	joints.Update(&s);
    logprintf("left_q=%f,right_q=%f,left_qmot=%f,right_qmot=%f,left_torque=%f,right_torque=%f",
	s.legs[0].knee.q,s.legs[1].knee.q,s.legs[0].kneemot.q,s.legs[1].kneemot.q,s.legs[0].knee.tau,s.legs[1].knee.tau);
	
	if (Controller()->TimeElapsed() >= 2.0)
		Controller()->Transition(&gKneeTest_StEnd);
}
