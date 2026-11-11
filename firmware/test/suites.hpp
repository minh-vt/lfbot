#pragma once

// One entry point per module. Each proves that module's contract against a
// stand-in plant; none proves behaviour on the robot.
void run_line();
void run_motion();
void run_robot();
