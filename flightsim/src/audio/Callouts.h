#pragma once
// S2: what a GPWS or minimums alert sounds like. With the clip in the voice bank it is a Voice
// event (spoken); without one (or with the voice off: bank = nullptr) it is the X2 tone. The App
// maps hud/GpwsMonitor events to AlertKind, so audio/ does not depend on the HUD.
#include "audio/AudioTypes.h"
#include "audio/VoiceBank.h"

namespace fsim::audio {

enum class AlertKind { Callout, SinkRate, PullUp, Terrain, TooLowGear, TooLowFlaps, Minimums };

// valueFt: the radio altitude of a Callout (2500 ... 10).
AudioEvent alertEvent(AlertKind kind, int valueFt, const VoiceBank* bank);

}  // namespace fsim::audio
