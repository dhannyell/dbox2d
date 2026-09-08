#include "textflag.h"

TEXT ·cpuPause(SB), NOSPLIT, $0-0
	PAUSE
	RET
