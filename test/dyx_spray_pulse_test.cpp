// Host-only state-machine tests. Built with g++ as part of the FMUv6X CI.
#include "src/lib/mixer_module/functions/DyxSprayPulse.hpp"

#include <cassert>

int main()
{
	DyxSprayPulse pulse;
	const uint64_t base = 1'000'000ULL;

	// Startup and unsafe request must be OFF.
	assert(pulse.update(base, false, true, true) == DyxSprayPulse::OFF);
	assert(pulse.update(base+1, true, true, true) == DyxSprayPulse::OFF);

	// Explicit OFF arms the next rising edge.
	assert(pulse.update(base+2, true, true, false) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3, true, true, true) == DyxSprayPulse::ON);
	assert(pulse.update(base+2'999'999, true, false, false) == DyxSprayPulse::ON);
	assert(pulse.update(base+3'000'003, true, false, false) == DyxSprayPulse::OFF);

	// Repeating the ON command does not retrigger when the pulse times out.
	assert(pulse.update(base+3'000'004, true, true, true) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'005, true, true, false) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'006, true, true, true) == DyxSprayPulse::ON);

	// An OFF command stops immediately, irrespective of 3-second budget.
	assert(pulse.update(base+3'000'007, true, true, false) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'008, true, true, true) == DyxSprayPulse::ON);

	// DDS/Offboard loss forces OFF and disarms pulse re-trigger.
	assert(pulse.update(base+3'000'009, false, false, false) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'010, true, true, true) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'011, true, true, false) == DyxSprayPulse::OFF);
	assert(pulse.update(base+3'000'012, true, true, true) == DyxSprayPulse::ON);

	return 0;
}
