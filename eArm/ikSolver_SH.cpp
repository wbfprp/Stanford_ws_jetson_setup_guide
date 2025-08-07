// ───── ikSolver_SH.cpp — 3‑DOF RR‑arm inverse kinematics + smooth motion (non‑blocking)
// Author: Seongheon Hong (2025‑07‑21)
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include "ikSolver_SH.h"          // IK_state_t enum / extern prototypes

// ───────────────────────────────── KINEMATICS CONSTANTS ──────────────────────
constexpr float a1 = 10.0f;     // shoulder Y‑offset  (mm)
constexpr float d1 = 76.5f;     // shoulder Z‑offset  (mm)
constexpr float L2 = 67.0f;     // link‑2 length      (mm)
constexpr float L3 = 120.0f;    // link‑3 length      (mm)

// joint limits (deg) FIND the right offset values!
constexpr float R1_OS = 2;   // mechanical zero offsets
constexpr float R2_OS = 120;
constexpr float R3_OS = 55;
constexpr float MIN1 = -R1_OS + 5, MAX1 = -R1_OS + 175;   // R1
constexpr float MIN2 = -R2_OS + 5, MAX2 = -R2_OS + 175;   // R2 
constexpr float MIN3 = -R3_OS + 5, MAX3 = -R3_OS + 175;   // R3 



// interpolation (ms)
constexpr uint16_t FRAME_MS = 20;
constexpr uint16_t DURATION_MS = 200;
constexpr uint16_t N_STEPS = DURATION_MS / FRAME_MS;

// ───────────────────────────────── GLOBAL STATE ──────────────────────────────
struct IkGoal { float x, y, z, g; bool pending; };
static volatile IkGoal goal = { 0,0,0, 90, false };

static hw_timer_t* timer = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t stepCnt = N_STEPS;   // start in “finished” state
static volatile float   aStart[3] = { 0 }, aDelta[3] = { 0 };
static volatile float   aCurr[4] = { 90,0,0,90 }; // t1,t2,t3,grip

// ──────────────────────────── Forward Declarations ───────────────────────────
static inline void sph2cart_xyElev(float r, float thetaDeg, float phiDeg,
                                   float &x, float &y, float &z);
static void setMode(bool isFastMode);

// ─────────────────────────── utility helpers ─────────────────────────────────
static inline bool withinLimits(float t1, float t2, float t3) {
    return (t1 >= MIN1 && t1 <= MAX1 &&
        t2 >= MIN2 && t2 <= MAX2 &&
        t3 >= MIN3 && t3 <= MAX3);
}

// ───────────────────────── analytic inverse kinematics ───────────────────────
bool solveIK(float x, float y, float z, float& t1, float& t2, float& t3) {
    /* 1. base yaw (rad) */
    float t1r = atan2f(y, x);
    if (t1r < MIN1 * DEG_TO_RAD || t1r > MAX1 * DEG_TO_RAD) return false;

    /* 2. shoulder frame */
    float rho = hypotf(x, y);
    float y1 = rho - a1;
    float z1 = z - d1;

    /* 3. elbow geometry */
    float r = hypotf(y1, z1);
    float c3 = (r * r - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if (c3 < -1.0f || c3 > 1.0f) return false;

    for (int s = +1; s >= -1; s -= 2) {
        float t3r = s * acosf(c3);                     // rad (+down, –up)
        float phi = atan2f(z1, y1);
        float psi = atan2f(L3 * sinf(t3r), L2 + L3 * cosf(t3r));
        float beta = phi - psi;
        float t2r = (90.0f * DEG_TO_RAD) - beta;      // flipped shoulder

        float t1d = t1r * RAD_TO_DEG;
        float t2d = -t2r * RAD_TO_DEG;                // extra minus (flip)*
        float t3d = -t3r * RAD_TO_DEG;                // extra minus (flip)

        if (withinLimits(t1d, t2d, t3d)) { t1 = t1d; t2 = t2d; t3 = t3d; return true; }
    }
    return false;                                   // neither branch OK
}

// ───────────────────────────── timer ISR (50 Hz) ─────────────────────────────
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&mux);
    if (stepCnt < N_STEPS) {
        for (int i = 0;i < 3;++i) aCurr[i] = aStart[i] + aDelta[i] * stepCnt;
        ++stepCnt;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

// ───────────────────────── public configuration API ─────────────────────────
void ikSetup() {
    timer = timerBegin(1000000);          
    timerAttachInterrupt(timer, &onTimer);  

    timerAlarm(timer, FRAME_MS * 1000, true, 0);
    // timer = timerBegin(1, 80, true);                 // 1 µs tick
    // timerAttachInterrupt(timer, &onTimer, true);
    // timerAlarmWrite(timer, FRAME_MS * 1000, true);
    // timerAlarmEnable(timer);
}

// set a new goal (non‑blocking, mailbox overwrite)
void ikSetGoal(float x, float y, float z, float gripDeg) {
    portENTER_CRITICAL(&mux);
    goal.x = x; goal.y = y; goal.z = z; goal.g = gripDeg; goal.pending = true;
    portEXIT_CRITICAL(&mux);
}


// -----------------------------------------------------------------------------
// set goal with spherical coords (non-blocking)
// -----------------------------------------------------------------------------
void ikSetGoalSpherical(float r, float thetaDeg, float phiDeg, float gripDeg)
{
    float x, y, z;
    sph2cart_xyElev(r, thetaDeg, phiDeg, x, y, z);
    ikSetGoal(x, y, z, gripDeg);   // reuse existing mailbox logic
}

// ───────────────────────── polling tick (call every loop) ───────────────────
IK_state_t ikTick(float& outA, float& outB, float& outC, float& outG) {
    /* 1. expose current angles */
    outA = aCurr[0] + R1_OS;
    outB = aCurr[1] + R2_OS;
    outC = aCurr[2] + R3_OS;
    outG = aCurr[3];

    /* 2. when interpolation done & new goal pending → arm next */
    if (stepCnt >= N_STEPS && goal.pending) {
        float t1, t2, t3;
        if (solveIK(goal.x, goal.y, goal.z, t1, t2, t3)) {
            portENTER_CRITICAL(&mux);
            aStart[0] = aCurr[0]; aDelta[0] = (t1 - aStart[0]) / (float)N_STEPS;
            aStart[1] = aCurr[1]; aDelta[1] = (t2 - aStart[1]) / (float)N_STEPS;
            aStart[2] = aCurr[2]; aDelta[2] = (t3 - aStart[2]) / (float)N_STEPS;
            aCurr[3] = goal.g;                     // grip instant set
            stepCnt = 0;
            goal.pending = false;
            portEXIT_CRITICAL(&mux);
            return IK_SUCCESS;
        }
        else {
            goal.pending = false;
            // Serial.println("IK failed");
            Serial1.println("IKF");
            return IK_FAILED;
        }
    }

    return (stepCnt < N_STEPS) ? IK_MOVING : IK_IDLE;
}

// -----------------------------------------------------------------------------
// Helper: spherical → Cartesian
// thetaDeg : azimuth around +Z (0° on +X, CCW toward +Y)
// phiDeg   : elevation from XY-plane (+up is +Z).  If you define phi from +Z,
//            swap sin/cos as noted below.
// -----------------------------------------------------------------------------
static inline void sph2cart_xyElev(float r, float thetaDeg, float phiDeg,
                                   float &x, float &y, float &z)
{
    float th = thetaDeg * DEG_TO_RAD;   // azimuth
    float ph = phiDeg   * DEG_TO_RAD;   // elevation from XY-plane
    // If phi is polar angle from +Z instead: 
    //   x = r * sinf(ph) * cosf(th);  y = r * sinf(ph) * sinf(th);  z = r * cosf(ph);
    x = r * cosf(ph) * cosf(th);
    y = r * cosf(ph) * sinf(th);
    z = r * sinf(ph);
}

