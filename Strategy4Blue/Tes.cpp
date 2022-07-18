#include "stdafx.h"
#include <string.h>
#include "Strategy4Blue.h"
#include <cmath>

void Tes(Environment *pEnv) {
	MoonAttack(&pEnv->home[1], pEnv);
	//Angle(&pEnv->home[0], 0);
	//Velocity(&pEnv->home[0],50,50);

}
void Tes2(Environment *pEnv) {
	//MoonAttack(&pEnv->home[4], pEnv);
	Position(&pEnv->home[3], 36, 85);

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
	if (env->currentBall.pos.x > 36 && env->currentBall.pos.x < 40)
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

double Distance(double x1, double x2, double y1, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void CenterDefender(Environment* pEnv, int id)
{
	Vector3D ball = pEnv->currentBall.pos;
	if (ball.x < 110)
	{
		Position3(pEnv, id, 200, 90);
	}
	else
	{
		if (ball.y > 115)
		{
			/*Position(pEnv, id, ball.x + 45, 90);
			if (ball.x + 45 > 200)
				Position(pEnv, id, 195, 90);*/
			Position3(pEnv, id, 200, 110);
		}
		else if (ball.y < 65)
		{
			/*Position(pEnv, id, ball.x + 45, 90);
			if (ball.x + 45 > 200)
				Position(pEnv, id, 195, 90);*/
			Position3(pEnv, id, 200, 75);
		}
		else
		{
			if (ball.x > 5)
			{
				Position3(pEnv, id, 200, ball.y);
			}
			else
			{
				Tes(pEnv);
			}
			
			//if (ball.x > 180 && ball.y > 50)
			//{
			//	//Position(pEnv, id, ball.x, 50);
			//	Tes(pEnv);
			//}
			//else if (ball.x>180 && ball.y > 130)
			//{
			//	//Position(pEnv, id, ball.x, 130);
			//	Tes(pEnv);
			//}
			////else
			//	//Position(pEnv, id, ball.x, ball.y);
		}
	}
}