// ───── ikSolver_SH.h ─────
// Author: Seongheon Hong (2025‑07‑21)
#ifndef IKSOLVER_SH_H
#define IKSOLVER_SH_H

#include <Arduino.h>

typedef enum { IK_MOVING = 0, IK_FAILED, IK_SUCCESS, IK_IDLE } IK_state_t;

void ikSetup();
void ikSetGoal(float x,float y,float z,float gripDeg);
void ikSetGoalSpherical(float x, float y, float z, float gripDeg);
IK_state_t ikTick(float &outA,float &outB,float &outC,float &outG);



#endif