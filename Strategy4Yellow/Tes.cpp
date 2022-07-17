#include "stdafx.h"
#include <stdlib.h>
#include <string.h>
#include "Strategy4Yellow.h"
#include <cmath>


void Tes(Robot *robot, Environment *pEnv) {
	MoonAttack(&pEnv->home[0], pEnv);

}
void Tes2(Environment *pEnv) {
	//MoonAttack(&pEnv->home[4], pEnv);
	Position(&pEnv->home[3], 181, 85);

}
void Goalie2(Robot *robot, Environment *pEnv)
{

	double xCentre = (pEnv->fieldBounds.left + 2);
	double ideal_pos = pEnv->currentBall.pos.y;

	if (ideal_pos > GTOPY) { ideal_pos = GTOPY; }
	else if (ideal_pos < GBOTY) { ideal_pos = GBOTY; }

	Position(robot, xCentre, ideal_pos);
}

void Jish_Defend1(Robot *robot, Environment *env)
{
	Vector3D ball_pos = env->currentBall.pos;
	OpponentRobot* target;
	double target_x = 100;
	for (int i = 0; i < 5; i++)
	{
		if (env->opponent[i].pos.x < target_x)
		{
			target = &env->opponent[i];
			target_x = env->opponent[i].pos.x;
		}
	}
	//MoonFollowOpponent(robot,target);
	//akshat
	double x, y;

	if (ball_pos.x > 50)
	{
		x = (ball_pos.x / 2) - 50;
		y = ball_pos.y + 1;
		Position(robot, x, y);
	}
	else if ((robot->pos.x > ball_pos.x - 3) && ball_pos.x < 50)
	{
		x = ball_pos.x - 3;
		if (ball_pos.y < 42)
			y = ball_pos.y - 2;

		if (ball_pos.y >= 42)
			y = ball_pos.y + 2;

		Position(robot, x, y);
	}
	if ((robot->pos.x <= ball_pos.x - 3) && robot->pos.x < 50)
	{
		x = ball_pos.x;
		y = ball_pos.y;
		Position(robot, x, y);
	}
}

void Jish_Defend2(Robot *robot, Environment *env)
{
	Vector3D ball_pos = env->currentBall.pos;
	OpponentRobot* buffer = nullptr;
	OpponentRobot* target = nullptr;
	double buffer_x = 100, target_x = 100;
	for (int i = 0; i < 5; i++)
	{
		if (env->opponent[i].pos.x < buffer_x)
		{
			target = buffer;
			target_x = buffer_x;
			buffer = &env->opponent[i];
			buffer_x = env->opponent[i].pos.x;
		}
		else if (env->opponent[i].pos.x < target_x)
		{
			target = &env->opponent[i];
			target_x = env->opponent[i].pos.x;
		}
	}

	//MoonFollowOpponent(robot, target);
	//by akshat

	double x, y;
	if (ball_pos.x > 50)
	{
		x = (ball_pos.x / 2) - 50;
		y = ball_pos.y - 1;
		Position(robot, x, y);
	}
	if ((robot->pos.x > ball_pos.x - 3) && ball_pos.x < 50)
	{
		x = ball_pos.x - 3;
		if (ball_pos.y < 42)
			y = ball_pos.y - 2;

		if (ball_pos.y >= 42)
			y = ball_pos.y + 2;
		Position(robot, x, y);
	}
	if ((robot->pos.x <= ball_pos.x - 3) && robot->pos.x < 50)
	{
		x = ball_pos.x;
		y = ball_pos.y;
		Position(robot, x, y);
	}
}

void Jish_Support(Robot *robot, Environment *env)
{
	double ball_dist = Distance(robot->pos.x, robot->pos.y, env->currentBall.pos.x, env->currentBall.pos.y);
	double goal_dist = Distance(robot->pos.x, robot->pos.y, FRIGHTX, (FBOT + (1 / 2)*FTOP));
	if (env->currentBall.pos.x > 181 && env->currentBall.pos.x < 183)
	{
		Tes2(env);
	}
	else if (ball_dist < 1 && goal_dist < 30)
	{
		Position(robot, FRIGHTX, (FBOT + (1 / 2)*FTOP));
	}
	else if (ball_dist < 1)
	{
		if (env->currentBall.pos.x < robot->pos.x) {
			double x_suggested = env->predictedBall.pos.x - 2;
			if (x_suggested < FLEFTX + (1 / 3)*FRIGHTX) { x_suggested = FLEFTX + (1 / 3)*FRIGHTX; }
			Position(robot, x_suggested, env->predictedBall.pos.y);
		}
		else
		{
			Position(robot, GRIGHT, GBOTY);
		}
	}
	else
	{
		double dx = env->currentBall.pos.x - env->lastBall.pos.x;
		double dy = env->currentBall.pos.y - env->lastBall.pos.y;
		double m1 = (dy / dx);
		double m2 = -(dx / dy);
		double k1 = env->currentBall.pos.y - (m1)*(env->currentBall.pos.x);
		double k2 = robot->pos.y - (m2)*(robot->pos.x);
		double x_return = (k2 - k1) / (m1 - m2);
		double y_return = m2 * (x_return)+k2;

		if (x_return < FLEFTX + (1 / 3)*FRIGHTX) { x_return = (FLEFTX + (1 / 3)*FRIGHTX); }

		Position(robot, x_return, y_return);
	}
}

void Jish_Attack(Robot *robot, Environment *env)
{
	//Get Distances to Ball
	double dist = Distance(env->currentBall.pos.x, env->currentBall.pos.y, robot->pos.x, robot->pos.y);

	/*if (robot->pos.x > (FLEFTX + (2/3)*FRIGHTX)){
		Position(robot,(FLEFTX + (2/3)*FRIGHTX),env->currentBall.pos.y);
	}
	else{*/
	if (env->currentBall.pos.x < robot->pos.x)
	{
		if (env->currentBall.pos.y > 42)
		{
			Position(robot, env->currentBall.pos.x - 3, env->currentBall.pos.y - 2);
		}
		else
		{
			Position(robot, env->currentBall.pos.x - 3, env->currentBall.pos.y + 2);
		}
	}
	else
	{
		if (dist > .1)
		{
			Position(robot, env->currentBall.pos.x, env->currentBall.pos.y);
		}
		else
		{
			Position(robot, GRIGHT, FTOP + (1 / 2)*FBOT);
		}
	}
}
void CenterDefender(Environment* pEnv, int id)
{
	Vector3D ball = pEnv->currentBall.pos;
	if (ball.x > 130)
	{
		Position3(pEnv, id, 20, 90);
	}
	else
	{
		if (ball.y > 115)
		{
			/*Position(pEnv, id, ball.x - 45, 90);
			if (ball.x - 45 < 20)
				Position(pEnv, id, 25, 90);*/
			Position3(pEnv, id, 20, 110);
		}
		else if (ball.y < 65)
		{
			/*Position(pEnv, id, ball.x - 45, 90);
			if (ball.x - 45 < 20)
				Position(pEnv, id, 25, 90);*/
			Position3(pEnv, id, 20, 70);
		}
		else
		{
			/*if (ball.x < 25)
			{
				Position(pEnv, id, 25, ball.y);
			}
			else
				Position(pEnv, id, ball.x, ball.y);*/
			Position3(pEnv, id, 20, ball.y);
		}
	}
}

double Distance(double x1, double x2, double y1, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void Striker1(int id, Environment * env)
{
	double strikeX, strikeY = 0.0;

	GetStrikePos(env, &strikeX, &strikeY);
	Robot * currentRobot = &env->home[id];

	MerlinPosition(currentRobot, strikeX, strikeY, 1.5);
}

void Striker2(int id, Environment * env)
{
	double strikeX, strikeY = 0.0;
	double strikeXt, strikeYt = 0.0;
	Robot * currentRobot = &env->home[id];
	double rx = currentRobot->pos.x;
	double ry = currentRobot->pos.y;
	double bx = env->currentBall.pos.x;
	double by = env->currentBall.pos.y;
	static int hf = 0;
	bool r = false;
	//MerlinPosition(currentRobot,strikeX,strikeY,1.5);
	if (hf == 0)
	{
		GetStrikePos(env, &strikeX, &strikeY);
		Robot * currentRobot = &env->home[id];

		ClipToBoundary(&strikeX, &strikeY, 2.0, 2.5);
		r = PositionRobot(id, env, strikeX, strikeY, 1);

		if (r == true)
		{
			hf = 1;
		}
	}
	if (hf == 1)
	{
		Strike(env, &strikeXt, &strikeYt);
		MerlinPosition(&env->home[id], strikeXt, strikeYt, 0.2);
	}
	if (bx > rx)
	{
		hf = 0;
	}
}

void Striker3(int id, Environment * env, int tolerance)
{
	double strikeX, strikeY = 0.0;

	GetStrikePos(env, &strikeX, &strikeY);
	Robot * currentRobot = &env->home[id];

	ClipToIntermediatePos(id, env, &strikeX, &strikeY);
	ClipToBoundary(&strikeX, &strikeY, 2.0, 2.5);
	MerlinPosition(currentRobot, strikeX, strikeY, tolerance);
}

void GetStrikePos(Environment * env, double * strikeX, double * strikeY)
{
	double strikeDistance = 6;
	double middleGoalY = GBOTY + (GTOPY - GBOTY) / 2;

	// distance from the ball to the opponent goal
	double dx = env->currentBall.pos.x - FLEFTX;
	// y-axis distance to the midpoint of the goal
	double dy = middleGoalY - env->currentBall.pos.y;
	// straight line distance to the goal midpoint
	double goalDist = sqrt(dx*dx + dy * dy);

	// ideal strike angle to the middle of the goal
	double strikeAngle = asin(dy / goalDist);

	// extend the ideal strike vector, strike distance beyond the ball position
	double Y = sin(strikeAngle)*strikeDistance;
	double X = cos(strikeAngle)*strikeDistance;

	// add the vectors to find the new position
	double sX = env->currentBall.pos.x + X;
	double sY = env->currentBall.pos.y - Y;

	if (env->currentBall.pos.y < FBOT + 2)
	{
		sY = env->currentBall.pos.y;
		sX = sX + 5;
	}

	if (env->currentBall.pos.y > FTOP - 2)
	{
		sY = env->currentBall.pos.y;
		sX = sX + 5;
	}

	if (sX > FRIGHTX + 1)
		sX = FRIGHTX - 1;

	*strikeX = sX;
	*strikeY = sY;
}
bool MerlinPosition(Robot *robot, double x, double y, double tolerance)
{
	double desiredAngle = 0;
	double dx = x - robot->pos.x;
	double dy = robot->pos.y - y;
	double kp = KP;
	double kd = KD;
	//double rob_rot = CorrectAngle((360-robot->rotation)+90);
	double rob_rot = CorrectAngle(360 - robot->rotation);
	double maxSpeed = MAX_SPEED;
	bool   backwards = false;
	double distance = sqrt(dx * dx + dy * dy);
	double maxDistance = sqrt(((FRIGHTX - FLEFTX)*(FRIGHTX - FLEFTX)) + ((FTOP - FBOT)*(FTOP - FBOT)));

	char buffer[255] = { 0 };

	if (fabs(distance) < tolerance)
	{
		Velocity(robot, (int)0, (int)0);
		OutputDebugString("Position reached\n");
		return true;
	}

	if (dx == 0 && dy == 0)
		desiredAngle = 90;
	else
	{
		double baseAngle = atan2(dy, dx)* (180.0 / PI);
		desiredAngle = CorrectAngle(baseAngle);
	}

	double delta = DiffAngle(desiredAngle, rob_rot);

	//	  sprintf((char*)&buffer,"RSE Angle: %.2f , Robot Angle: %.2f , Desired Angle: %.2f , Delta Angle: %.2f\n", robot->rotation, rob_rot,desiredAngle, delta);	
	//	OutputDebugString(buffer);

	if ((delta > 90) || (delta < -90))
	{
		backwards = true;
		rob_rot = CorrectAngle(180 + rob_rot);   // make back to front
		delta = DiffAngle(desiredAngle, rob_rot);
	}

	if ((fabs(delta) > 45) && (distance < 3))
	{
		OutputDebugString("Big angular error short distance .. lets just rotate?\n");
	}

	// clip the max speed at a quarter of the distance
	maxDistance = maxDistance / 4;
	if (distance > maxDistance)
		distance = maxDistance;

	// modify the max speed with distance
	double distanceFactor = exp(distance*kd) / exp(maxDistance*kd);
	double turnFactor = exp(fabs(delta)*kp) / exp(45.0*kp);

	// MAX_TURN_FACTOR prioritise turning over speed
	double baseSpeed = (maxSpeed*(1 - MAX_TURN_FACTOR))*distanceFactor;
	double turnSpeed = (maxSpeed*MAX_TURN_FACTOR)*turnFactor;

	if (backwards)
	{
		baseSpeed = -baseSpeed;
		//turnSpeed = -turnSpeed;
	}
	turnSpeed = (delta / fabs(delta))*turnSpeed;

	Velocity(robot, (int)(baseSpeed + turnSpeed), (int)(baseSpeed - turnSpeed));
	//Velocity(robot, (int) 10, (int)10);

	return false;
}
void ClipToBoundary(double * x, double * y, double border, double tol)
{

	if (*x > FRIGHTX - tol)
		*x = FRIGHTX - border;
	if (*x < FLEFTX + tol)
		*x = FLEFTX + border;
	if (*y > FTOP - tol)
		*y = FTOP - border;
	if (*y < FBOT + tol)
		*y = FBOT + border;

}
bool PositionRobot(int id, Environment * env, double x, double y, double tolerance)
{
	Robot * robot = &env->home[id];
	return MerlinPosition(robot, x, y, tolerance);
}
void Strike(Environment * env, double * strikeXt, double * strikeYt)
{
	double strikeDistance = 2;
	double middleGoalY = GBOTY + (GTOPY - GBOTY) / 2;

	// distance from the ball to the opponent goal
	double dx = env->currentBall.pos.x - FLEFTX;
	// y-axis distance to the midpoint of the goal
	double dy = middleGoalY - env->currentBall.pos.y;
	// straight line distance to the goal midpoint
	double goalDist = sqrt(dx*dx + dy * dy);

	// ideal strike angle to the middle of the goal
	double strikeAngle = asin(dy / goalDist);

	// extend the ideal strike vector, strike distance beyond the ball position
	double Y = sin(strikeAngle)*strikeDistance;
	double X = cos(strikeAngle)*strikeDistance;

	// add the vectors to find the new position
	double sXt = env->currentBall.pos.x + X;
	double sYt = env->currentBall.pos.y - Y;

	if (env->currentBall.pos.y < FBOT + 2)
	{
		sYt = env->currentBall.pos.y;
		sXt = sXt + 5;
	}

	if (env->currentBall.pos.y > FTOP - 2)
	{
		sYt = env->currentBall.pos.y;
		sXt = sXt + 5;
	}

	if (sXt > FRIGHTX + 1)
		sXt = FRIGHTX - 1;

	*strikeXt = sXt;
	*strikeYt = sYt;
}
void ClipToIntermediatePos(int id, Environment * env, double * x, double * y)
{
	Robot * currentRobot = &env->home[id];
	double rx = currentRobot->pos.x;
	double ry = currentRobot->pos.y;
	double bx = env->currentBall.pos.x;
	double by = env->currentBall.pos.y;
	double disp = 6.0; // displacement above or below the ball

	// We are on the left hand side of the ball
	// And the target position is on the right hand side ?
	if ((rx < bx) && (*x > bx))
	{
		// We need an intermediate position to go around the ball
		// We need to figure out wether to go below or above

		if (by < MIDDLEY)
		{
			// ball is in the bottom half -> let's go above it
			*x = bx + disp / 2;
			*y = by + disp;
		}
		else
		{
			// ball is in the bottom half -> let's go above it
			*x = bx + disp / 2;
			*y = by - disp;
		}



	}

}
double DiffAngle(double alpha, double beta)
{
	double swap, result = 0;
	bool neg = false;

	if (alpha > 180)
		alpha = 360 - alpha;
	else if (alpha < -180)
		alpha = alpha + 360;

	if (beta > 180)
		beta = 360 - beta;
	else if (beta < -180)
		beta = beta + 360;

	if (alpha < beta)
	{
		swap = alpha;
		alpha = beta;
		beta = swap;
		neg = true;
	}

	if ((alpha > 0) && (beta < 0))
	{
		if ((alpha - beta) > 180) 			// alpha + beta (because beta is neg.)
			result = -((180 - alpha) + (180 + beta));
		else
			result = alpha - beta;

	}
	else
		result = alpha - beta;       //normal case (both negative or positive)

	if (neg)
		result = -result;        			// alpha < beta

	return result;
}
double CorrectAngle(double angle)
{

	while (angle > 180)
		angle = -(360 - angle);
	while (angle <= -180)
		angle = (360 + angle);

	return angle;
}