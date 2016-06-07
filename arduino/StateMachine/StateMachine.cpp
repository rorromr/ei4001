#include "StateMachine.h"

namespace statemachine
{
	StateMachine::StateMachine(Encoder& encoder, PID& pid, HBridge& hbridge, Fin_de_Carrera& fdc):
	encoder_(&encoder),
	pid_(&pid),
	hbridge_(&hbridge),
	fdc_(&fdc)
	{
		; 
	}

	StateMachine::init()
	{
	switch (state_)
	{
		case UPDATE:


	}
	}

	StateMachine::check()
	{

	}

	StateMachine::error_soft()
	{

	}

	StateMachine::error_fatal()
	{

	}

	StateMachine::move()
	{

	}

	StateMachine::getState()
	{

	}
}
