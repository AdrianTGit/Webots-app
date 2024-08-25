#include <stdlib.h>
class MyArena {
	public:
		MyArena(int x, int z){
			cols = x;
			rows = z;
			B = new bool *[x];
			for (short i = 0; i < x; i++)
				B[i] = new bool[z];
		}
		unsigned short int cols;
		unsigned short int rows;
		bool **B; // matrice care arata prezenta unui obstacol pentru fiecare casuta
		//seteaza obstacol in casuta x, z
		void SetObstacle (unsigned short int x, unsigned short int z){
			B[x][z] = true;
		}
		//input: coordonate webots in metri, multiplicate cu 10
		//output: coordonate arena
		unsigned short int GetX(int x1)
		{
			return (cols-1+x1)/2;
		}
		unsigned short int GetZ(int z1)
		{
			return (rows-1+z1)/2;
		}
		//adauga obstacole pentru obiectul de la posX, posZ cu laturile lX, lZ
		void addNode(short int posX, short int posZ, short int lX, short int lZ)
		{
			std::cout<<"addNode "<<posX<<", "<<posZ<<", "<<lX<<", "<<lZ<<std::endl;
			short int x1 = posX - lX/2 + 1;
			short int x2 = posX + lX/2 - 1;
			std::cout<<"x1 "<<x1<<", x2 "<<x2<<std::endl;
			for(short int ix = x1; ix <= x2; ix += 2)
			{
				std::cout<<"for addNode "<<ix<<std::endl;
				for(short int iz = posZ - lZ/2 + 1; iz <= (posZ + lZ/2 - 1); iz += 2)
				{
					SetObstacle(GetX(ix), GetZ(iz));
				}
			}
		}
};
//AStarNode defineste nodurile folosite in algoritmul AStar 
class AStarNode{
	public:
		unsigned short int X; //coordonata X a casutei din matrice 
		unsigned short int Z;//coordonata X a casutei din matrice
		unsigned short int Previous; //indicele nodului anterior din array-ul closed
		unsigned short int G; //functia G din algoritmul AStar reprezinta costul de a ajunge pana in aceasta casuta
		unsigned short int F; //functia F = G + H
		unsigned short int Dir; //directia din care s-a ajuns in acest nod
								//0=NE, 1=E, 2=SE, 3=S, 4=SV, 5=V, 6=NV, 7=N
		AStarNode(unsigned short int x, unsigned short int z, unsigned short int prev, unsigned short int g, unsigned short int f, unsigned short int d){
			X = x;
			Z = z;
			Previous = prev;
			F = f;
			G = g;
			Dir = d;
		}
		AStarNode(){
			Dir = 0;
		}
};


class Pathfinding{
	bool **map;
	unsigned short int meX, meZ, targetX, targetZ, openCount, closedCount, cols, rows;
	//std::vector<AStarNode *> open;
	//std::vector<AStarNode *> closed;
	AStarNode openList[1560];
	AStarNode closed[1560];
	//functia H valoare euristica reprezinta costul minim pana la casuta finala
	unsigned short int H(int x, int z){
		unsigned short int a = abs(x-targetX);
		unsigned short int b = abs(z-targetZ);
		if(a > b)
			return (141 * b + 100 * (a - b) + ((a != b && a != 0 && b != 0) ? 1 : 0));
		else
			return (141 * a + 100 * (b - a) + ((a != b && a != 0 && b != 0) ? 1 : 0));
	}
	//primul dintre cele mai promitatoare noduri din open, pentru care valoarea functiei F este minima
	unsigned short int Promising(unsigned short int openCount){
		unsigned short int p = 0;
		for	(int i = 1; i < openCount; i++)
		{
			if (openList[i].F < openList[p].F)
			{
				p = i;
			}
		}
		std::cout<<"promising is "<<p<<std::endl;
		return p;
	}
	//mutarea unui nod din open in closed
	void CloseNode(unsigned short int nodeNr){
		std::cout<<"closing "<<nodeNr<<std::endl;
		closed[closedCount] = openList[nodeNr];
		closedCount++;
		std::cout<<"closed count "<<closedCount<<std::endl;
		openCount--;
		std::cout<<"open count "<<openCount<<std::endl;
		if (nodeNr < openCount){
			openList[nodeNr] = openList[openCount];
		}
	}
	//explorarea nodurilor adiacente ultimului nod inchis
	void Explore(){
		short int moveX [] = {1, 1, 1, 0, -1, -1, -1, 0};
		short int moveZ [] = {-1, 0, 1, 1, 1, 0, -1, -1};
		AStarNode nd = closed[closedCount-1];
		std::cout<<nd.X<<", "<<nd.Z<<", "<<nd.G<<", "<<nd.F<<", "<<nd.Dir<<std::endl;
		for	(int d = 0; d < 8; d++){
			std::cout<<"d is "<<d<<std::endl;
			unsigned short int g;
			unsigned short int dda = abs (d - nd.Dir);
			if (dda > 2 && dda < 6){
				std::cout<<"failed curve unaccepted"<<std::endl;
				continue;
			}
			short int newX = nd.X + moveX[d];
			short int newZ = nd.Z + moveZ[d];
			if (newX < 0 || newX >= cols || newZ < 0 || newZ >= rows){
				std::cout<<"failed outside arena"<<std::endl;
				continue;}
			if (d % 2 == 0){
				if(map[newX][newZ] || map[nd.X][newZ] || map[newX][nd.Z]){
					std::cout<<"failed diagonal"<<std::endl;
					continue;
				}
				g = 141;
			}
			else if(map[newX][newZ])
			{
				std::cout<<"failed obstacle"<<std::endl;
				continue;
			}
			else
				g = 100;
			bool inClosed = false;
			for	(int i = 0; i < closedCount; i++)
			{
				if (newX == closed[i].X && newZ == closed[i].Z)
				{
					std::cout<<"failed in closed"<<std::endl;
					inClosed = true;
					break;
				}
			}
			if (inClosed)
				continue;
			bool inOpen = false;
			g += (dda < 2 ? dda : 8 - dda);
			for	(int i = 0; i < openCount; i++)
			{
				if (newX == openList[i].X && newZ == openList[i].Z)
				{
					inOpen = true;
					if (nd.G + g < openList[i].G)
					{
						std::cout<<"update "<<openList[i].X<<", "<<openList[i].Z<<", "<<openList[i].G<<", "<<openList[i].F<<", "<<openList[i].Dir<<std::endl;
						openList[i].Previous = closedCount-1;
						openList[i].G = nd.G + g;
						openList[i].F = nd.G + g + H(openList[i].X, openList[i].Z);
						openList[i].Dir = d;
						std::cout<<"updated "<<openList[i].X<<", "<<openList[i].Z<<", "<<openList[i].G<<", "<<openList[i].F<<", "<<openList[i].Dir<<std::endl;
					}
					break;
				}
			}
			if (!inOpen)
			{
			openList[openCount] = AStarNode(nd.X + moveX[d], nd.Z + moveZ[d], closedCount - 1, nd.G + g, nd.G + g + H(nd.X + moveX[d], nd.Z + moveZ[d]), d);
			std::cout<<openList[openCount].X<<", "<<openList[openCount].Z<<", "<<openList[openCount].G<<", "<<openList[openCount].F<<", "<<openList[openCount].Dir<<std::endl;
			openCount++;
			}
		}
	}
	void FirstExplore(){
		short int moveX [] = {1, 1, 1, 0, -1, -1, -1, 0};
		short int moveZ [] = {-1, 0, 1, 1, 1, 0, -1, -1};
		AStarNode nd = closed[closedCount-1];
		std::cout<<nd.X<<", "<<nd.Z<<", "<<nd.G<<", "<<nd.F<<", "<<nd.Dir<<std::endl;
		for	(int d = 0; d < 8; d++){
			std::cout<<"d is "<<d<<std::endl;
			unsigned short int g;
			if (nd.X + moveX[d] < 0 || nd.X + moveX[d] >= cols || nd.Z + moveZ[d] < 0 || nd.Z + moveZ[d] >= rows){
				std::cout<<"failed 1"<<std::endl;
				continue;}
			if (d % 2 == 0){
				if(map[nd.X + moveX[d]][nd.Z + moveZ[d]] || map[nd.X][nd.Z + moveZ[d]] || map[nd.X + moveX[d]][nd.Z]){
					std::cout<<"failed 2"<<std::endl;
					continue;
				}
				g = 141;
			}
			else if(map[nd.X + moveX[d]][nd.Z + moveZ[d]])
			{
				std::cout<<"failed 3"<<std::endl;
				continue;
			}
			else
				g = 100;
			openList[openCount] = AStarNode(nd.X + moveX[d], nd.Z + moveZ[d], closedCount - 1, g, g + H(nd.X + moveX[d], nd.Z + moveZ[d]), d);
			std::cout<<openList[openCount].X<<", "<<openList[openCount].Z<<", "<<openList[openCount].G<<", "<<openList[openCount].F<<", "<<openList[openCount].Dir<<std::endl;
			openCount++;
		}
	}
	void BuildPath(AStarNode target)
	{
		AStarNode asn = target;
		while (asn.X != meX || asn.Z != meZ)
		{
			goldenPathX[pathCount] = asn.X;
			goldenPathZ[pathCount] = asn.Z;
			orientation[pathCount] = asn.Dir;
			std::cout<<asn.X<<","<<asn.Z<<" ; "<<asn.Dir<<" ;F "<<asn.F<<" ;G "<<asn.G<<std::endl;
			pathCount++;
			asn = closed[asn.Previous];
		}
		/*for	(int j = pathCount-1 ; j >= 0; j--)
		{
			std::cout<<goldenPathX[j]<<","<<goldenPathZ[j]<<" ; "<<orientation[j]<<std::endl;
		}*/
	}
	public:
		unsigned short int pathCount;
		unsigned short int goldenPathX[830];
		unsigned short int goldenPathZ[830];
		unsigned short int orientation[830];
		Pathfinding(bool **B, unsigned short int startX, unsigned short int startZ, unsigned short int endX, unsigned short int endZ, unsigned short int dx, unsigned short int dz){
			std::cout<<"ctor "<<startX<<", "<<startZ<<", "<<endX<<", "<<endZ<<std::endl;
			map = B;
			meX = startX;
			meZ = startZ;
			targetX = endX;
			targetZ = endZ;
			openCount = 0, closedCount = 0, pathCount = 0;
			cols = dx;
			rows = dz;
		}
		bool Astar(){
			std::cout<<meX<<", "<<meZ<<", "<<targetX<<", "<<targetZ<<std::endl;
			bool fe = true;
			AStarNode current;
			unsigned short int crtNr;
			openList[openCount] = AStarNode(meX, meZ, 1199, 0, H(meX, meZ), 7);
			openCount++;
			while (openCount > 0){
				std::cout<<"openCount = "<<openCount<<std::endl;
				crtNr = Promising(openCount);
				current = openList[crtNr];
				if (current.X == targetX && current.Z == targetZ){
					std::cout<<"TARGET! BRAVO!"<<std::endl;
					BuildPath(current);
					return true;
				}
				std::cout<<current.X<<", "<<current.Z<<", "<<current.G<<", "<<current.F<<", "<<current.Dir<<std::endl;
				CloseNode(crtNr);
				if (fe){
					FirstExplore();
					fe = false;
				}
				else
				{
					Explore();
				}
			}
			return false;
		}
		
};