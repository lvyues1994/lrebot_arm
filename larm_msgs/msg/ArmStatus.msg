# State of the arm as seen by the runtime.

uint8 SAFETY_NORMAL = 0
uint8 SAFETY_FAULTED = 1
uint8 SAFETY_EMERGENCY_STOPPED = 2

builtin_interfaces/Time stamp
# Actuators on and the stiffness ramp finished.
bool enabled
# Every joint reported fresh feedback in the latest control cycle.
bool feedback_fresh
uint8 safety
# Latched fault, e.g. "feedback_lost"; "none" when safety is normal.
string fault
# Joint groups that currently run a goal.
string[] active_groups
# Why the most recent rejected or failed goal ended; empty if none has.
string last_error
