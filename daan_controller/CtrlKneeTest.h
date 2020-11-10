#ifndef __CTRLKNEETEST_H_INCLUDED
#define __CTRLKNEETEST_H_INCLUDED

#include "StateMachines.h"
#include "globals.h"
#include <hardware_drivers/FlameIO.h>
#include "polynomials.h"
#include "LookupTables.h"
#include <utility/utility.h>


class CKneeTestController: public CStateMachine
{
	public:
		void	Init();
};

class CKneeTest_StEnd: public CStateMachineState
{

};


class CKneeTest_StKnee: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

extern CKneeTestController	gKneeTestController;
extern CKneeTest_StEnd		gKneeTest_StEnd;

#endif