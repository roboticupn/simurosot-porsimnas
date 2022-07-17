// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the STRATEGY4BLUE_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// STRATEGY4BLUE_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef STRATEGY4BLUE_EXPORTS
#define STRATEGY4BLUE_API __declspec(dllexport)
#else
#define STRATEGY4BLUE_API __declspec(dllimport)
#endif


#include <math.h>

const long PLAYERS_PER_SIDE = 5;
//const double PI = 3.1415926535;
const double PI = 3.1415923;

//GLOBAL VARIABLES idk it's gonna be useful or not
const double FTOP = 77.2392;
const double FBOT = 6.3730;
const double GTOPY = 49.6801;
const double GBOTY = 33.9320;
const double GRIGHT = 97.3632;
const double GLEFT = 2.8748;
const double FRIGHTX = 93.4259;
const double FLEFTX = 6.8118;

typedef struct
{
	double x, y, z;
} Vector3D;


typedef struct
{
	long left, right, top, bottom;
} Bounds;                              


typedef struct
{
	Vector3D pos;
	double rotation;
	double velocityLeft, velocityRight;
} Robot;


typedef struct
{
	Vector3D pos;
	double rotation;
} OpponentRobot;                      


typedef struct
{
	Vector3D pos;
} Ball;


typedef struct
{
	Robot home[PLAYERS_PER_SIDE];
	OpponentRobot opponent[PLAYERS_PER_SIDE];
	Ball currentBall, lastBall, predictedBall;
	Bounds fieldBounds, goalBounds;
	long gameState;
	long whosBall;
	void *userData;
} Environment;


enum PlayMode {
	PM_PlayOn = 0,           //  
	PM_FreeBall_LeftTop,     //   1
	PM_FreeBall_LeftBot,     //   2
	PM_FreeBall_RightTop,    //   3
	PM_FreeBall_RightBot,    //   4
	PM_PlaceKick_Yellow,     //   5
	PM_PlaceKick_Blue,       //   6
	PM_PenaltyKick_Yellow,   //   7
	PM_PenaltyKick_Blue,     //   8
	PM_FreeKick_Yellow,      //   9
	PM_FreeKick_Blue,        //   10
	PM_GoalKick_Yellow,      //   11
	PM_GoalKick_Blue         //   12
};


// #############################################################################################
STRATEGY4BLUE_API void SetFormerRobots(PlayMode gameState, Robot robots[]);
STRATEGY4BLUE_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[], Vector3D ball, Robot laterRobots[]);
STRATEGY4BLUE_API void SetBall(PlayMode gameState, Vector3D * pBall); 
STRATEGY4BLUE_API void RunStrategy(Environment *pEnv);
STRATEGY4BLUE_API void SetBlueTeamName(char* teamName);

// #############################################################################################
void PredictBall(Environment *pEnv);
void Goalie1(Robot *robot, Environment *pEnv);
void NearBound2(Robot *robot, double vl, double vr, Environment *pEnv);
void Attack2(Robot *robot, Environment *pEnv);
void Defend(Robot *robot, Environment *pEnv, double low, double high);
void MoonAttack(Robot *robot, Environment *pEnv);
void MoonFollowOpponent(Robot *robot, OpponentRobot *opponent);
void Velocity(Robot *robot, double vl, double vr);
void Angle(Robot *robot, double desired_angle);
void Position(Robot *robot, double x, double y);
void Position(Environment * pEnv, int id, double x, double y);
void Position2(Robot *robot, double x, double y);
void Position2(Environment * pEnv, int id, double x, double y);
void Position3(Robot *robot, double x, double y);
void Position3(Environment * pEnv, int id, double x, double y);

void Tes(Environment *pEnv);
void Tes2(Environment *pEnv);
void Goalie(Environment* pEnv);
void LeftWing(Environment* pEnv, int id);
void RightWing(Environment* pEnv, int id);
void CenterDefender(Environment* pEnv, int id);
void CenterAttacker(Environment* pEnv, int id);
void Jish_Defend1(Robot *robot, Environment *env);
void Jish_Defend2(Robot *robot, Environment *env);
void Jish_Support(Robot *robot, Environment *env);
void Jish_Attack(Robot *robot, Environment *env);
double Distance(double x1, double x2, double y1, double y2);



// #############################################################################################