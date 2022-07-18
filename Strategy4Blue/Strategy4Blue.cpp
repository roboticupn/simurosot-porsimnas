// Strategy4Blue.cpp  
//

#include "stdafx.h"
#include <string.h>
#include "Strategy4Blue.h"
#include <cmath>


// Set the team name for blue side.
STRATEGY4BLUE_API void SetBlueTeamName(char* teamName)
{
	// MUST change the name into your own team name!
	strcpy(teamName, "Manut Blue");
}

// Set the positions and rotations of your team when your team places robots first.
// [IN] gameState : the state of game
// [OUT] robots : robots data for placing,including position and rotation
STRATEGY4BLUE_API void SetFormerRobots(PlayMode gameState, Robot robots[])
{
	// This is just for a demo. You may set your own data.
	switch (gameState){
	case 1:
		break;
	case 2:
		break;
	case 3:
		robots[0].pos.x = 219;
		robots[0].pos.y = 100;
		robots[0].rotation = -90;
		robots[1].pos.x = 200;
		robots[1].pos.y = 90;
		robots[1].rotation = -90;
		robots[2].pos.x = 198;
		robots[2].pos.y = 80;
		robots[2].rotation = 180;
		robots[3].pos.x = 160;
		robots[3].pos.y = 80;
		robots[3].rotation = 180;
		robots[4].pos.x = 195;
		robots[4].pos.y = 150;
		robots[4].rotation = 180;
		break;
	case 4:
		robots[0].pos.x = 219;
		robots[0].pos.y = 80;
		robots[0].rotation = -90;
		robots[1].pos.x = 200;
		robots[1].pos.y = 90;
		robots[1].rotation = -90;
		robots[2].pos.x = 198;
		robots[2].pos.y = 100;
		robots[2].rotation = 180;
		robots[3].pos.x = 160;
		robots[3].pos.y = 100;
		robots[3].rotation = 180;
		robots[4].pos.x = 195;
		robots[4].pos.y = 30;
		robots[4].rotation = 180;
		break;
	case 5:		
		break;
	case 6:		
		robots[0].pos.x = 219;
		robots[0].pos.y = 90;
		//robots[0].rotation = 90;
		robots[0].rotation = -90;
		robots[1].pos.x = 200;
		robots[1].pos.y = 90;
		robots[1].rotation = -90;
		robots[2].pos.x = 120;
		robots[2].pos.y = 90;
		robots[2].rotation = 180;
		robots[3].pos.x = 110;
		robots[3].pos.y = 80;
		robots[3].rotation = 90;
		robots[4].pos.x = 120;
		robots[4].pos.y = 100;
		robots[4].rotation = 180;
		/*robots[0].pos.x = 215;
		robots[0].pos.y = 90;
		robots[0].rotation = -90;
		robots[1].pos.x = 150;
		robots[1].pos.y = 90;
		robots[1].rotation = 180;
		robots[2].pos.x = 150;
		robots[2].pos.y = 100;
		robots[2].rotation = 180;
		robots[3].pos.x = 150;
		robots[3].pos.y = 80;
		robots[3].rotation = 180;
		robots[4].pos.x = 95;
		robots[4].pos.y = 90;
		robots[4].rotation = 180;*/
		break;
	case 7:
		robots[0].pos.x = 219;
		robots[0].pos.y = 90;
		robots[0].rotation = -90;
		robots[1].pos.x = 70;
		robots[1].pos.y = 160;
		robots[1].rotation = 180;
		robots[2].pos.x = 70;
		robots[2].pos.y = 120;
		robots[2].rotation = 180;
		robots[3].pos.x = 70;
		robots[3].pos.y = 80;
		robots[3].rotation = 180;
		robots[4].pos.x = 70;
		robots[4].pos.y = 50;
		robots[4].rotation = 180;
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		break;
	case 11:
		break;
	case 12:
		robots[0].pos.x = 219;
		robots[0].pos.y = 90;
		robots[0].rotation = -90;
		robots[1].pos.x = 200;
		robots[1].pos.y = 90;
		robots[1].rotation = -90;
		robots[2].pos.x = 170;
		robots[2].pos.y = 65;
		robots[2].rotation = 180;
		robots[3].pos.x = 180;
		robots[3].pos.y = 40;
		robots[3].rotation = 180;
		robots[4].pos.x = 215;
		robots[4].pos.y = 110;
		robots[4].rotation = 180;
		break;
	}
}

// Set the positions and rotations of your team when your team places robots last.
// [IN] gameState : the state of game
// [IN] formerRobots : the data of first placing team
// [IN] ball : the position of the ball
// [OUT] laterRobots : robots data for placing,including position and rotation
STRATEGY4BLUE_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[], 
	Vector3D ball, Robot laterRobots[])
{	
	// This is just for a demo. You may set your own data.
	switch (gameState){
	case 1:
		laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 130;
		laterRobots[2].pos.y = 120;
		laterRobots[2].rotation = 180;
		laterRobots[3].pos.x = 95;
		laterRobots[3].pos.y = 80;
		laterRobots[3].rotation = 180;
		laterRobots[4].pos.x = 85;
		laterRobots[4].pos.y = 150;
		laterRobots[4].rotation = 180;
		break;
	case 2:
		laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 130;
		laterRobots[2].pos.y = 60;
		laterRobots[2].rotation = 180;
		laterRobots[3].pos.x = 95;
		laterRobots[3].pos.y = 100;
		laterRobots[3].rotation = 180;
		laterRobots[4].pos.x = 85;
		laterRobots[4].pos.y = 30;
		laterRobots[4].rotation = 180;
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		/*laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 120;
		laterRobots[2].pos.y = 70;
		laterRobots[2].rotation = 180;
		laterRobots[3].pos.x = 130;
		laterRobots[3].pos.y = 90;
		laterRobots[3].rotation = 180;
		laterRobots[4].pos.x = 120;
		laterRobots[4].pos.y = 110;
		laterRobots[4].rotation = 180;*/

		laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 150;
		laterRobots[2].pos.y = 90;
		laterRobots[2].rotation = 180;
		laterRobots[3].pos.x = 130;
		laterRobots[3].pos.y = 60;
		laterRobots[3].rotation = 180;
		laterRobots[4].pos.x = 130;
		laterRobots[4].pos.y = 120;
		laterRobots[4].rotation = 180;
		break;
	case 6:		
		break;
	case 7:
		break;
	case 8:
		laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 115;
		laterRobots[2].pos.y = 130;
		laterRobots[2].rotation = 180;
		laterRobots[3].pos.x = 36;
		laterRobots[3].pos.y = 20;
		laterRobots[3].rotation = 90;
		laterRobots[4].pos.x = 150;
		laterRobots[4].pos.y = 90;
		laterRobots[4].rotation = 180;

		/*laterRobots[0].pos.x = 215;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 180;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = 180;
		laterRobots[2].pos.x = 115;
		laterRobots[2].pos.y = 110;
		laterRobots[2].rotation = 225;
		laterRobots[3].pos.x = 115;
		laterRobots[3].pos.y = 120;
		laterRobots[3].rotation = 180;
		laterRobots[4].pos.x = 75;
		laterRobots[4].pos.y = 88;
		laterRobots[4].rotation = 135;*/
		break;	
	case 9:
		break;
	case 10:
		break;
	case 11:
		laterRobots[0].pos.x = 219;
		laterRobots[0].pos.y = 90;
		laterRobots[0].rotation = -90;
		laterRobots[1].pos.x = 200;
		laterRobots[1].pos.y = 90;
		laterRobots[1].rotation = -90;
		laterRobots[2].pos.x = 140;
		laterRobots[2].pos.y = 130;
		laterRobots[2].rotation = 180;
		laterRobots[4].pos.x = 120;
		laterRobots[4].pos.y = 30;
		laterRobots[4].rotation = 180;
		laterRobots[3].pos.x = 120;
		laterRobots[3].pos.y = 90;
		laterRobots[3].rotation = 180;
		break;
	case 12:		
		break;
	}
}

// Set the position of the ball when your team get the Goal Kick.
// [IN] gameState : the state of game
// [IN] pBall : the position of the ball
STRATEGY4BLUE_API void SetBall(PlayMode gameState, Vector3D * pBall)
{
	// This is just for a demo. You may set your own data.
	if (PM_GoalKick_Blue == gameState)
	{
		pBall->x = 210;
		pBall->y = 110;
	}
}

// Strategy for your team using centimeter unit, cartesian coordinate system.
// The origin is at the left bottom of the field.
STRATEGY4BLUE_API void RunStrategy(Environment *pEnv)
{

	Goalie(pEnv); //(pake)
	/*Goalie3(&pEnv->home[0],pEnv);*/
	CenterDefender(pEnv, 1); //(pake)
	//RightWing(pEnv, 2);
	//CenterDefender(pEnv, 2);
	Jish_Attack(&pEnv->home[2], pEnv); //(pake)
	//CenterAttacker(pEnv, 4);
	//Attack2(&pEnv->home[4], pEnv);
	//Tes(&pEnv->home[0], pEnv);
	//Jish_Defend1(&pEnv->home[1], pEnv);
	//Jish_Defend2(&pEnv->home[2], pEnv);
	Jish_Support(&pEnv->home[3],pEnv);
	/*CenterAttacker(pEnv, 3);*/ //(pake);
	Jish_Attack(&pEnv->home[4], pEnv); //(pake)
	//Tes3(pEnv);
	//Tes4(pEnv);
}



void Goalie(Environment* pEnv)
{
	/*if (pEnv->gameState == 7) {
		MoonAttack(&pEnv->home[0], pEnv);
	}*/
	/*if (pEnv->currentBall.pos.x > 181 && pEnv->currentBall.pos.y < 115 && pEnv->currentBall.pos.y > 65)
	{
		Tes(pEnv);

	}
	else*/ if (pEnv->currentBall.pos.x < 110)
	{
		Position2(pEnv, 0, 219, 90);
	}
	else
	{
		/*if (pEnv->currentBall.pos.y == 90) {
			Position(pEnv, 0, 210, 110);
		}
		else*/ if (pEnv->currentBall.pos.y > 115)
		{
			Position2(pEnv, 0, 219, 110);
		}
		else if (pEnv->currentBall.pos.y < 65)
		{
			Position2(pEnv, 0, 220, 75);
		}
		else
		{
			Position2(pEnv, 0, 219, pEnv->currentBall.pos.y);
			/*if (pEnv->currentBall.pos.x > 208) { Position2(pEnv, 0, pEnv->currentBall.pos.x, pEnv->currentBall.pos.y);
				Angle(&pEnv->home[0], 0);
				Tes(pEnv);
			}*/
			

		}
	}
}

void LeftWing(Environment* pEnv, int id)
{
	Vector3D ball = pEnv->currentBall.pos;
	if (ball.y < 45)
	{
		Position(pEnv, id, ball.x, ball.y);
	}
	else if (ball.y > 135)
	{
		Position(pEnv, id, ball.x + 8, 60);
	}
	else
	{
		if (ball.x > 195)
		{
			if (ball.y > 100)
				Position(pEnv, id, 20, 30);
			else if (ball.y > 40)
				Position(pEnv, id, 20, 60);
			else
				Position(pEnv, id, ball.x, ball.y);
		}
		else
		{
			if (ball.y < 80)
				Position(pEnv, id, ball.x, ball.y);
			else
				Position(pEnv, id, ball.x + 20, 40);
		}
	}
}

void RightWing(Environment* pEnv, int id)
{
	Vector3D ball = pEnv->currentBall.pos;
	if (ball.y < 45)
	{
		Position(pEnv, id, ball.x + 8, 120);
	}
	else if (ball.y > 135)
	{
		Position(pEnv, id, ball.x, ball.y);
	}
	else
	{
		if (ball.x > 195)
		{
			if (ball.y < 80)
				Position(pEnv, id, 20, 150);
			else if (ball.y < 140)
				Position(pEnv, id, 20, 120);
			else
				Position(pEnv, id, ball.x, ball.y);
		}
		else
		{
			if (ball.y > 80)
				Position(pEnv, id, ball.x, ball.y);
			else
				Position(pEnv, id, ball.x + 20, 140);
		}
	}
}

//void CenterDefender(Environment* pEnv, int id)
//{
//	Vector3D ball = pEnv->currentBall.pos;
//	if (ball.x < 90)
//	{
//		Position(pEnv, id, 110, 90);
//	}
//	else
//	{
//		if (ball.y > 130)
//		{
//			Position(pEnv, id, ball.x + 45, 90);
//			if (ball.x + 45 > 200)
//				Position(pEnv, id, 195, 90);
//		}
//		else if (ball.y < 50)
//		{
//			Position(pEnv, id, ball.x + 45, 90);
//			if (ball.x + 45 > 200)
//				Position(pEnv, id, 195, 90);
//		}
//		else
//		{
//			if (ball.x > 195)
//			{
//				Position(pEnv, id, 195, ball.y);
//			}
//			//else
//				//Position(pEnv, id, ball.x, ball.y);
//		}
//	}	
//}

void CenterAttacker(Environment* pEnv, int id)
{
	Vector3D ball = pEnv->currentBall.pos;
	if (ball.x < 45)
		Position(pEnv, id, 45, 90);
	else
	{
		if (ball.y > 160)
		{
			Position(pEnv, id, ball.x - 45, 110);
			if (ball.x - 45 < 45)
				Position(pEnv, id, 45, 110);
		}
		else if (ball.y < 20)
		{
			Position(pEnv, id, ball.x - 45, 70);
			if (ball.x - 45 < 45)
				Position(pEnv, id, 20, 70);
				//Tes3(pEnv);
		}
		else
		{
			Position(pEnv, id, ball.x, ball.y);
			//Tes3(pEnv);
		}
	}
}


void MoonAttack(Robot *robot, Environment *pEnv)
{
	Angle(&pEnv->home[0], 0);
	PredictBall(pEnv);
	Position(robot, pEnv->predictedBall.pos.x, pEnv->predictedBall.pos.y);
}

void MoonFollowOpponent(Robot *robot, OpponentRobot *opponent)
{
	Position(robot, opponent->pos.x, opponent->pos.y);
}

void Velocity(Robot *robot, double vl, double vr)
{
	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

void Angle(Robot *robot, double desired_angle)
{
	double theta_e = 0, vl = 0, vr = 0;
	theta_e = desired_angle - robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (theta_e < -90) theta_e += 180;

	else if (theta_e > 90) theta_e -= 180;

	if (abs(theta_e) > 50)
	{
		vl = -9. / 90.0 * theta_e;
		vr = 9. / 90.0 * theta_e;
	}
	else if (abs(theta_e) > 20)
	{
		vl = -11.0 / 90.0 * theta_e;
		vr = 11.0 / 90.0 * theta_e;
	}
	else {
		vl = -13.0 / 90.0 * theta_e;
		vr = 13.00 / 90.0 * theta_e;
	}
	Velocity(robot, vl, vr);
}

void Position(Environment* pEnv, int id, double x, double y)
{
	Position(&(pEnv->home[id]), x, y);
}

void Position(Robot *robot, double x, double y)
{
	double desired_angle = 0, theta_e = 0, d_angle = 0;
	double vl = 0, vr = 0, vc = 180;

	double dx, dy, d_e, Ka = 10.0 / 90.0;
	dx = x - robot->pos.x;
	dy = y - robot->pos.y;
	d_e = sqrt(dx * dx + dy * dy);

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = 180. / PI * atan2((double)(dy), (double)(dx));
	theta_e = desired_angle - robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.)
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else
		Ka = 25. / 90.;

	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;

		if (theta_e > 180)
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (+.17 * theta_e);
		vl = (-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}
void Position2(Environment* pEnv, int id, double x, double y)
{
	Position2(&(pEnv->home[id]), x, y);
}

void Position2(Robot *robot, double x, double y)
{
	double desired_angle = 0, theta_e = 0, d_angle = 0;
	double vl = 0, vr = 0, vc = 125;

	double dx, dy, d_e, Ka = 10.0 / 90.0;
	dx = x - robot->pos.x;
	dy = y - robot->pos.y;
	d_e = sqrt(dx * dx + dy * dy);

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = 180. / PI * atan2((double)(dy), (double)(dx));
	theta_e = desired_angle - robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.)
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else
		Ka = 25. / 90.;

	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;

		if (theta_e > 180)
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (+.17 * theta_e);
		vl = (-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}
void Position3(Environment* pEnv, int id, double x, double y)
{
	Position3(&(pEnv->home[id]), x, y);
}

void Position3(Robot *robot, double x, double y)
{
	double desired_angle = 0, theta_e = 0, d_angle = 0;
	double vl = 0, vr = 0, vc = 30;

	double dx, dy, d_e, Ka = 10.0 / 90.0;
	dx = x - robot->pos.x;
	dy = y - robot->pos.y;
	d_e = sqrt(dx * dx + dy * dy);

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = 180. / PI * atan2((double)(dy), (double)(dx));
	theta_e = desired_angle - robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.)
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else
		Ka = 25. / 90.;

	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;

		if (theta_e > 180)
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (+.17 * theta_e);
		vl = (-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}


void PredictBall(Environment *pEnv)
{
	double dx = pEnv->currentBall.pos.x - pEnv->lastBall.pos.x;
	double dy = pEnv->currentBall.pos.y - pEnv->lastBall.pos.y;
	pEnv->predictedBall.pos.x = pEnv->currentBall.pos.x + dx;
	pEnv->predictedBall.pos.y = pEnv->currentBall.pos.y + dy;
}

void Goalie1(Robot *robot, Environment *pEnv)
{
	double velocityLeft = 0, velocityRight = 0;

	double Tx = pEnv->goalBounds.right - pEnv->currentBall.pos.x;
	double Ty = pEnv->fieldBounds.top - pEnv->currentBall.pos.y;

	double Ax = pEnv->goalBounds.right - robot->pos.x;
	double Ay = pEnv->fieldBounds.top - robot->pos.y;

	if (Ay > Ty + 0.9 && Ay > 27)
	{
		velocityLeft = -100;
		velocityRight = -100;
	}

	if (Ay > Ty - 0.9 && Ay < 43)
	{
		velocityLeft = 100;
		velocityRight = 100;
	}

	if (Ay < 27)
	{
		velocityLeft = 100;
		velocityRight = 100;
	}

	if (Ay > 43)
	{
		velocityLeft = -100;
		velocityRight = -100;
	}

	double Tr = robot->rotation;
	if (Tr < 0.001)
		Tr = Tr + 360;
	if (Tr > 360.001)
		Tr = Tr - 360;
	if (Tr > 270.5)
		velocityRight = velocityRight + fabs(Tr - 270);
	else if (Tr < 269.5)
		velocityLeft = velocityLeft + fabs(Tr - 270);

	robot->velocityLeft = velocityLeft;
	robot->velocityRight = velocityRight;
}

void Attack2(Robot *robot, Environment *pEnv)
{
	Vector3D t = pEnv->currentBall.pos;
	double r = robot->rotation;
	if (r < 0) r += 360;
	if (r > 360) r -= 360;
	double vl = 0, vr = 0;

	if (t.y > pEnv->fieldBounds.top - 2.5) t.y = pEnv->fieldBounds.top - 2.5;
	if (t.y < pEnv->fieldBounds.bottom + 2.5) t.y = pEnv->fieldBounds.bottom + 2.5;
	if (t.x > pEnv->fieldBounds.right - 3) t.x = pEnv->fieldBounds.right - 3;
	if (t.x < pEnv->fieldBounds.left + 3) t.x = pEnv->fieldBounds.left + 3;

	double dx = robot->pos.x - t.x;
	double dy = robot->pos.y - t.y;

	double dxAdjusted = dx;
	double angleToPoint = 0;

	if (fabs(robot->pos.y - t.y) > 7 || t.x > robot->pos.x)
		dxAdjusted -= 5;

	if (dxAdjusted == 0)
	{
		if (dy > 0)
			angleToPoint = 270;
		else
			angleToPoint = 90;
	}
	else if (dy == 0)
	{
		if (dxAdjusted > 0)
			angleToPoint = 360;
		else
			angleToPoint = 180;

	}
	else
		angleToPoint = atan(fabs(dy / dx)) * 180.0 / PI;

	if (dxAdjusted > 0)
	{
		if (dy > 0)
			angleToPoint -= 180;
		else if (dy < 0)
			angleToPoint = 180 - angleToPoint;
	}
	if (dxAdjusted < 0)
	{
		if (dy > 0)
			angleToPoint = -angleToPoint;
		else if (dy < 0)
			angleToPoint = 90 - angleToPoint;
	}

	if (angleToPoint < 0) angleToPoint = angleToPoint + 360;
	if (angleToPoint > 360) angleToPoint = angleToPoint - 360;
	if (angleToPoint > 360) angleToPoint = angleToPoint - 360;

	double c = r;

	double angleDiff = fabs(r - angleToPoint);

	if (angleDiff < 40)
	{
		vl = 100;
		vr = 100;
		if (c > angleToPoint)
			vl -= 10;
		if (c < angleToPoint)
			vr -= 10;
	}
	else
	{
		if (r > angleToPoint)
		{
			if (angleDiff > 180)
				vl += 360 - angleDiff;
			else
				vr += angleDiff;
		}
		if (r < angleToPoint)
		{
			if (angleDiff > 180)
				vr += 360 - angleDiff;
			else
				vl += angleDiff;
		}
	}

	NearBound2(robot, vl, vr, pEnv);
}

void NearBound2(Robot *robot, double vl, double vr, Environment *pEnv)
{
	Vector3D a = robot->pos;
	double r = robot->rotation;

	if (a.y > pEnv->fieldBounds.top - 15 && r > 45 && r < 130)
	{
		if (vl > 0)
			vl /= 3;
		if (vr > 0)
			vr /= 3;
	}

	if (a.y < pEnv->fieldBounds.bottom + 15 && r < -45 && r > -130)
	{
		if (vl > 0) vl /= 3;
		if (vr > 0) vr /= 3;
	}

	if (a.x > pEnv->fieldBounds.right - 10)
	{
		if (vl > 0)
			vl /= 2;
		if (vr > 0)
			vr /= 2;
	}

	if (a.x < pEnv->fieldBounds.left + 10)
	{
		if (vl > 0)
			vl /= 2;
		if (vr > 0)
			vr /= 2;
	}

	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

void Defend(Robot *robot, Environment *pEnv, double low, double high)
{
	double vl = 0, vr = 0;
	Vector3D z = pEnv->currentBall.pos;

	double Tx = pEnv->goalBounds.right - z.x;
	double Ty = pEnv->fieldBounds.top - z.y;
	Vector3D a = robot->pos;
	a.x = pEnv->goalBounds.right - a.x;
	a.y = pEnv->fieldBounds.top - a.y;

	if (a.y > Ty + 0.9 && a.y > low)
	{
		vl = -100;
		vr = -100;
	}
	if (a.y < Ty - 0.9 && a.y < high)
	{
		vl = 100;
		vr = 100;
	}
	if (a.y < low)
	{
		vl = 100;
		vr = 100;
	}
	if (a.y > high)
	{
		vl = -100;
		vr = -100;
	}

	double Tr = robot->rotation;

	if (Tr < 0.001)
		Tr += 360;
	if (Tr > 360.001)
		Tr -= 360;
	if (Tr > 270.5)
		vr += fabs(Tr - 270);
	else if (Tr < 269.5)
		vl += fabs(Tr - 270);

	NearBound2(robot, vl, vr, pEnv);
}