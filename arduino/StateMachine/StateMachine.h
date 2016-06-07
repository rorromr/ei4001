#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>

namespace statemachine
{
	class StateMachine
	{
	public:
		StateMachine(Encoder& encoder, PID& pid, HBridge& hbridge, Fin_de_Carrera& fdc);

		init();

		check();

		error_soft();

		error_fatal();

		move();

		getState();

	private:
		Encoder* encoder_;
		PID* pid_;
		HBridge* hbridge_;
		Fin_de_Carrera* fdc_;
	};
}
