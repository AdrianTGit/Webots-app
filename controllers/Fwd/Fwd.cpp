


#include <webots/supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Node.hpp>
#include <libraryA.cpp>
#define TIME_STEP 64
#define PRECISION_MARGIN 0.0475

using namespace webots;
using namespace std; // nu este necesar,deoarece exista in bibliotecile webots

void moveFwd(Robot *robotel);
void rotateR(Robot *robotel);
void rotateL(Robot *robotel);
void stop(Robot *robotel);
void HandleNode (Node *nod, MyArena arena);
void AddShape(Node *nod, short int posX, short int posZ, MyArena arena);
double GetXCoord(unsigned short int posX);
double GetZCoord(unsigned short int posZ);

int main(int argc, char **argv) {
  // create the Robot instance.
  
	Supervisor *robot = new Supervisor();
	Node *arena = robot->getFromDef("ARENA");
	Field *aSize = arena->getField("floorSize");
	const double *sizeVector = aSize->getSFVec2f();
	int columns = sizeVector[0]*5;
	int rows = sizeVector[1]*5;
	std::cout<<"Arena are "<<rows<<" linii si "<<columns<<" coloane"<<std::endl;
	MyArena theArena(columns,rows);
	Node *grupAll = robot->getFromDef("GALL");
	HandleNode(grupAll, theArena);
	Field *nodulete = grupAll->getField("children");
	int count = nodulete->getCount();
	unsigned short int woodenBoxes = 0;
	unsigned short int solids = 0;
	for (int i = 0; i < count; i++)
	{
		string tipul = nodulete->getMFNode(i)->getTypeName();
		if (tipul == "WoodenBox")
			woodenBoxes ++;
		else if (tipul == "Solid")
			solids ++;
	}
	std::cout<<count<<" obstacole, "<<woodenBoxes<<" de tip woodenBox si "<<solids<<" de tip solid"<<std::endl;
	GPS *gps = robot->getGPS("global");
	gps->enable(TIME_STEP);
	Compass *busola = robot->getCompass("busola");
	busola->enable(TIME_STEP);
	Node *roby = robot->getFromDef("ROBY");
	double oldX = roby->getPosition()[0];
	double oldZ = roby->getPosition()[2];
	unsigned short int oldDirection = 7;
	Node *goal = robot->getFromDef("GOAL");
	double goalX = goal->getPosition()[0];
	double goalZ = goal->getPosition()[2];
	//std::cout<<"goal "<<goalX<<". "<<goalZ<<std::endl;
	Pathfinding searchA = Pathfinding(theArena.B, theArena.GetX(10 * oldX), theArena.GetZ(10 * oldZ), theArena.GetX(10 * goalX), theArena.GetZ(10 * goalZ), columns, rows);
	bool foundPath = searchA.Astar();
	unsigned short int stepsLeft = 999;
	if (foundPath)
		stepsLeft = searchA.pathCount;
	else{
		stop(robot);  
		std::cout<<"Nu s-a gasit o cale pana la tinta! "<<std::endl;
		delete robot;
		return 0;
	}
	Motor *leftMotor = robot->getMotor("left wheel motor");
	Motor *rightMotor = robot->getMotor("right wheel motor");
	leftMotor->setPosition(999);
	rightMotor->setPosition(999);
	int nrS = 2;
	nrS++;
	nrS++;
  
	int timeStep = (int)robot->getBasicTimeStep();

  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

	double nextX = 0;
	double nextZ = 0;
	unsigned short int direction = 7;
	bool newStep = true;
	bool rotate = false;
	bool gata = false;
	bool rotated = false;
	while (robot->step(timeStep) != -1) {	
	double x = gps->getValues()[0];
	double z = gps->getValues()[2];
	double bx = busola->getValues()[0];
	double bz = busola->getValues()[2];
	if (bx != bx){
		stop(robot);
		continue;
	}
	if (gata)
	{
		stop(robot);
		continue;
	}
	if (newStep && stepsLeft < 830)
	{
		newStep = false;
		unsigned short int xuletz = searchA.goldenPathX[stepsLeft - 1];
		unsigned short int zuletz = searchA.goldenPathZ[stepsLeft - 1];
	  //std::cout<<"uletz "<<xuletz<<", "<<zuletz<<", "<<stepsLeft<<std::endl;
		nextX = GetXCoord(xuletz);
		nextZ = GetZCoord(zuletz);
		direction = searchA.orientation[stepsLeft - 1];
		rotated = false;
	}
  
	if((direction == 5 && bz < .99982) || (direction == 1 && bz > -.99982) || (direction == 7 && bx < .99982) || (direction == 3 && bx > -.99982) || (direction == 6 && (bx < .704 || bz < .704)) ||
		(direction == 4 && (bx > -.704 || bz < .704)) || (direction == 0 && (bx < .704 || bz > -.704)) || (direction == 2 && (bx > -.704 || bz > -.704)))
		rotate = true;
	else{
		rotate = false;
		rotated = true;
	}
	//std::cout<<"rotate "<<", "<<rotate<<std::endl;
	short int changeDirection = direction - oldDirection;
	if (changeDirection < -3)
		changeDirection +=8;
	if ((changeDirection > 4))
		changeDirection -=8;
	if(rotate){
		if (direction != oldDirection && !rotated){
		//std::cout<<"changeDirection  "<<changeDirection<<std::endl;
		if(changeDirection == -1 || changeDirection == -2 || changeDirection == -3)
			rotateL (robot);
		else if(changeDirection == 1 || changeDirection == 2 || changeDirection == 3 || changeDirection == 4)
			rotateR (robot);
		}
		else
		{
			if(direction == 5){
				if(bx > 0)
					rotateL (robot);
				else
					rotateR (robot);
			}
			if(direction == 1){
				if(bx < 0)
					rotateL (robot);
				else
					rotateR (robot);
			}
			if(direction == 7){
				if(bz < 0)
					rotateL (robot);
				else
					rotateR (robot);
			}
			if(direction == 3){
				if(bz > 0)
					rotateL (robot);
				else
					rotateR (robot);
			}
		}
	}
	else
	{
		if((abs(x - nextX) < PRECISION_MARGIN) && (abs(z - nextZ) < PRECISION_MARGIN)){
			stop(robot);
			if(stepsLeft == 1)
			{
				leftMotor->setPosition(0);
				rightMotor->setPosition(0);
				std::cout<<"Gata, am ajuns! "<<std::endl;
				gata = true;
				delete robot;
				return 0;
			}
			newStep = true;
			stepsLeft--;
			oldX = nextX;
			oldZ = nextZ;
			oldDirection = direction;
		}			  
		else
			moveFwd (robot);
	}
};

delete robot;
return 0;
}
void moveFwd(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(3.6);
  rightMotor->setVelocity(3.6);
}
//rotire la dreapta, putere in roata stanga
void rotateR(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0.4);
  rightMotor->setVelocity(0);
}
//rotire la stanga, putere in roata dreapta
void rotateL(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0.4);
}
void stop(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}
//citire nod webots
void HandleNode (Node *nod, MyArena arena)
{
	std::string tip = nod->getTypeName();
	short int posX;
	short int posZ;
	short int sizeX;
	short int sizeZ;
	if(tip=="WoodenBox"){
		posX = 10*(nod->getPosition()[0]);
		posZ = 10*nod->getPosition()[2];
		Field *marimea = nod->getField("size");
		const double *valori = marimea->getSFVec3f();
		sizeX = 10*valori[0];
		sizeZ = 10*valori[2];
		arena.addNode(posX, posZ, sizeX, sizeZ);
	}
	else if(tip=="Group")
	{
		Field *nodulete = nod->getField("children");
		int count = nodulete->getCount();
		for (int i = 0; i < count; i++)
		{
			Node *ni = nodulete->getMFNode(i);
			HandleNode (ni,  arena);
		}
	}
	else if(tip=="Solid" || tip=="Transform")
	{
		posX = 10*(nod->getPosition()[0]);
		posZ = 10*nod->getPosition()[2];
		Field *nodulete = nod->getField("children");
		int count = nodulete->getCount();
		for (int i = 0; i < count; i++)
		{
			Node *ni = nodulete->getMFNode(i);
			if(ni->getTypeName() == "Shape")
			{
				AddShape(ni, posX, posZ, arena);
			}
			else if(ni->getTypeName() == "Group")
			{
				Field *kids = ni->getField("children");
				int kidsCount = kids->getCount();
				for (int j = 0; j < kidsCount; j++)
				{
					Node *kiddo = kids->getMFNode(j);
					if(kiddo->getTypeName() == "Shape")
					{
						AddShape(kiddo, posX, posZ, arena);
					}
					else
						HandleNode (kiddo,  arena);
				}
			}
			else
				HandleNode (ni,  arena);
		}
	}
}
void AddShape(Node *nod, short int posX, short int posZ, MyArena arena)
{
	Field *geom = nod->getField("geometry");
	Node *cutie = geom-> getSFNode();
	Field *boxSize = cutie->getField("size");
	const double *xyzSize = boxSize->getSFVec3f();
	short int sizeX = 10*xyzSize[0];
	short int sizeZ = 10*xyzSize[2];
	arena.addNode(posX, posZ, sizeX, sizeZ);
}
double GetXCoord(unsigned short int posX)
{
	return -3.9 + .2 * posX;
}
double GetZCoord(unsigned short int posZ)
{
	return -2.9 + .2 * posZ;
}