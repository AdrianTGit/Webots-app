// File:          Fwd.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Node.hpp>
#include <libraryA.cpp>
//#include <webots/Solid.hpp>
#define TIME_STEP 64
#define PRECISION_MARGIN 0.0475
// All the webots classes are defined in the "webots" namespace
using namespace webots;
void moveFwd(Robot *robotel);
void rotateR(Robot *robotel);
void rotateL(Robot *robotel);
void processA (bool **array, size_t rows, size_t cols);
void sit(Robot *robotel);
void HandleNode (Node *nod, MyArena arena);
void AddShape(Node *nod, short int posX, short int posZ, MyArena arena);
double GetXCoord(unsigned short int posX);
double GetZCoord(unsigned short int posZ);
//template <size_t cols, size_t rows>
//void processTA(bool (&array)[cols][rows]);

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  //WbNodeRef node = wb_supervisor_node_get_from_def("solid1.solid1B");
  // create the Robot instance.
  Supervisor *robot = new Supervisor();
  Node *arena = robot->getFromDef("ARENA");
  Field *aSize = arena->getField("floorSize");
  const double *sizeVector = aSize->getSFVec2f();
  int columns = sizeVector[0]*5;
  int rows = sizeVector[1]*5;
  Node *nodul = robot->getFromDef("WALL4");
  std::cout<<"rows "<<rows<<" cols "<<columns<<std::endl;
  MyArena theArena(columns,rows);
  Field *copii = nodul->getField("children");
  Node *grup4 = copii->getMFNode(0);
  Field *noduri = grup4->getField("children");
  Node *altNod = noduri->getMFNode(0);
  Node *shape4 = noduri->getMFNode(1);
  std::cout<<nodul->getPosition()[0]<<std::endl;
  std::cout<<altNod->getPosition()[0]<<std::endl;
  std::cout<<altNod->getTypeName()<<std::endl;
  std::cout<<nodul->getTypeName()<<std::endl;
  std::cout<<grup4->getTypeName()<<std::endl;
  std::cout<<shape4->getTypeName()<<std::endl;
  Field *geom = shape4->getField("geometry");
  Node *cutie = geom-> getSFNode();
  Field *marimea = cutie->getField("size");
  const double *valori = marimea->getSFVec3f();
  std::cout<<valori[0]<<std::endl;
  Node *grupAll = robot->getFromDef("GALL");
  HandleNode(grupAll, theArena);
  Field *nodulete = grupAll->getField("children");
  int count = nodulete->getCount();
  std::cout<<count<<" noduri"<<std::endl;
  for (int i=0; i<count; i++)
  {
    std::cout<<nodulete->getMFNode(i)->getTypeName()<<std::endl;
  }
  GPS *gps = robot->getGPS("global");
  gps->enable(TIME_STEP);
  Compass *busola = robot->getCompass("busola");
  busola->enable(TIME_STEP);
  Node *roby = robot->getFromDef("ROBY");
  double oldX = roby->getPosition()[0];
  double oldZ = roby->getPosition()[2];
  unsigned short int oldDirection = 7;
  std::cout<<oldX<<". "<<oldZ<<std::endl;
  Node *goal = robot->getFromDef("GOAL");
  double goalX = goal->getPosition()[0];
  double goalZ = goal->getPosition()[2];
  std::cout<<"goal "<<goalX<<". "<<goalZ<<std::endl;
  Pathfinding searchA = Pathfinding(theArena.B, theArena.GetX(10 * oldX), theArena.GetZ(10 * oldZ), theArena.GetX(10 * goalX), theArena.GetZ(10 * goalZ), columns, rows);
  bool foundPath = searchA.Astar();
  unsigned short int stepsLeft = 999;
  if (foundPath)
	  stepsLeft = searchA.pathCount;
  else{
	sit(robot);  
	std::cout<<"Gata, am ajuns! "<<std::endl;
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
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  double nextX;
  double nextZ;
  unsigned short int direction;
  bool newStep = true;
  bool rotate = false;
  bool gata = false;
  bool rotated = false;
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
  double x = gps->getValues()[0];
  //double y = gps->getValues()[1];
  double z = gps->getValues()[2];
  //std::cout<<x<<", "<<z<<std::endl;
  double bx = busola->getValues()[0];
  //double by = busola->getValues()[1];
  double bz = busola->getValues()[2];
  //std::cout<<"Compass "<<bx<<", "<<bz<<std::endl;
  if (bx != bx){
	  sit(robot);
	  continue;
  }
  if (gata)
  {
	  sit(robot);
	  continue;
  }
  if (newStep && stepsLeft < 830)
  {
	  newStep = false;
	  unsigned short int xuletz = searchA.goldenPathX[stepsLeft - 1];
	  unsigned short int zuletz = searchA.goldenPathZ[stepsLeft - 1];
	  std::cout<<"uletz "<<xuletz<<", "<<zuletz<<", "<<stepsLeft<<std::endl;
	  nextX = GetXCoord(xuletz);
	  nextZ = GetZCoord(zuletz);
	  direction = searchA.orientation[stepsLeft - 1];
	  std::cout<<oldX<<", "<<oldZ<<", "<<oldDirection<<", "<<nextX<<", "<<nextZ<<", "<<direction<<std::endl;
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
			std::cout<<"changeDirection  "<<changeDirection<<std::endl;
			if(changeDirection == -1 || changeDirection == -2 || changeDirection == -3)
				rotateL (robot);
			else if(changeDirection == 1 || changeDirection == 2 || changeDirection == 3 || changeDirection == 4)
				rotateR (robot);
		}
		else
		{
			std::cout<<"fine direction tunning "<<std::endl;
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
			  sit(robot);
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
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
void moveFwd(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(3.6);
  rightMotor->setVelocity(3.6);
  //std::cout<<"moving forward"<<std::endl;
}
void rotateR(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0.4);
  rightMotor->setVelocity(0);
  //std::cout<<"rotating right"<<std::endl;
}
void rotateL(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0.4);
  //std::cout<<"rotating left"<<std::endl;
}
void sit(Robot *robotel)
{
  Motor *leftMotor = robotel->getMotor("left wheel motor");
  Motor *rightMotor = robotel->getMotor("right wheel motor");
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  //std::cout<<"sit"<<std::endl;
}
void processA (bool **array, size_t rows, size_t cols)
{
	array[4][3] = true;
}
template <size_t cols, size_t rows>
void processTA(bool (&array)[cols][rows])
{
	array[2][2] = true;
}
void HandleNode (Node *nod, MyArena arena)
{
	std::string tip = nod->getTypeName();
	std::cout<<"hn tip "<<tip<<std::endl;
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
		std::cout<<"case WB "<<posX<<", "<<posZ<<", "<<sizeX<<", "<<sizeZ<<std::endl;
		arena.addNode(posX, posZ, sizeX, sizeZ);
	}
	else if(tip=="Group")
	{
		Field *nodulete = nod->getField("children");
		int count = nodulete->getCount();
		std::cout<<count<<" noduri"<<std::endl;
		for (int i = 0; i < count; i++)
		{
			//std::cout<<nodulete->getMFNode(i)->getTypeName()<<std::endl;
			Node *ni = nodulete->getMFNode(i);
			HandleNode (ni,  arena);
		}
	}
	else if(tip=="Solid" || tip=="Transform")
	{
		posX = 10*(nod->getPosition()[0]);
		posZ = 10*nod->getPosition()[2];
		std::cout<<"case "<<tip<<" "<<posX<<", "<<posZ<<std::endl;
		Field *nodulete = nod->getField("children");
		int count = nodulete->getCount();
		std::cout<<count<<" noduri"<<std::endl;
		for (int i = 0; i < count; i++)
		{
			//std::cout<<nodulete->getMFNode(i)->getTypeName()<<std::endl;
			Node *ni = nodulete->getMFNode(i);
			if(ni->getTypeName() == "Shape")
			{
				AddShape(ni, posX, posZ, arena);
			}
			else if(ni->getTypeName() == "Group")
			{
				Field *kids = ni->getField("children");
				int kidsCount = kids->getCount();
				std::cout<<kidsCount<<" kids"<<std::endl;
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
	//else if(tip=="Transform")
	//{
		
	//}
}
void AddShape(Node *nod, short int posX, short int posZ, MyArena arena)
{
	Field *geom = nod->getField("geometry");
	Node *cutie = geom-> getSFNode();
	Field *boxSize = cutie->getField("size");
	const double *xyzSize = boxSize->getSFVec3f();
	short int sizeX = 10*xyzSize[0];
	short int sizeZ = 10*xyzSize[2];
	std::cout<<"case shape "<<posX<<", "<<posZ<<", "<<sizeX<<", "<<sizeZ<<std::endl;
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