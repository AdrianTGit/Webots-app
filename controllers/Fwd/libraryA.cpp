#include <stdlib.h>

using namespace std; // nu este necesar,deoarece exista in bibliotecile webots

// clasa pentru crearea matricii care transforma coordonatele webots in coordonate similare unei matrici ( 0,0;0,1...)
class MyArena {
	public:
		MyArena(int x, int z){ // x si y reprezinta coordonatele matricii
			cols = x; // numarul de coloane
			rows = z; //numarul de linii
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
			short int x1 = posX - lX/2 + 1;
			short int x2 = posX + lX/2 - 1;
			for(short int ix = x1; ix <= x2; ix += 2)
			{
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
		unsigned short int Previous; //indicele nodului anterior din array-ul closed (nodul TATA)
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
	
	// meX si meZ reprezinta pozitia curenta a robotului
	// targetX si targetZ reprezinta pozitia in care trebuie sa ajunga robotul	
	AStarNode openList[1560];  // 1560 reprezinta numarul maxim de noduri (52x30)
	AStarNode closed[1560];
	// in momentul de fata arena are dimensiunea de 40x30 = 1200 de noduri posibile
	
	//functia H valoare euristica reprezinta costul minim pana la casuta finala
	unsigned short int H(int x, int z){
		unsigned short int a = abs(x-targetX);
		unsigned short int b = abs(z-targetZ);
		if(a > b)
			return (141 * b + 100 * (a - b));
		else
			return (141 * a + 100 * (b - a));
	}
	
	// 141 reprezinta costul miscarii pe diagonala iar 100 reprezinta costul miscarii pe orizontala sau verticala conform euristicii utilizate
	
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
		return p;
	}
	//mutarea unui nod din open in closed
	void CloseNode(unsigned short int nodeNr){
		closed[closedCount] = openList[nodeNr];
		closedCount++;
		openCount--;
		if (nodeNr < openCount){
			openList[nodeNr] = openList[openCount];
		}
	}
	//explorarea nodurilor adiacente ultimului nod inchis
	void Explore(){
		short int moveX [] = {1, 1, 1, 0, -1, -1, -1, 0}; // coordonatele nodului din jurul nodului curent
		short int moveZ [] = {-1, 0, 1, 1, 1, 0, -1, -1}; // coordonatele nodului din jurul nodului curent
		AStarNode nd = closed[closedCount-1];
		
		// X - coordonata coloanei X a robotului
		// Z - coordonata randului Z a robotului
		// G - costul de a ajunge in patratelul curent
		// F - functia F corespunzatoare A*
		// Dir - directia robotului
		
		for	(int d = 0; d < 8; d++){
			unsigned short int g;
			unsigned short int dda = abs (d - nd.Dir);
			if (dda > 2 && dda < 6){
				continue;
			}
			short int newX = nd.X + moveX[d];
			short int newZ = nd.Z + moveZ[d];
			if (newX < 0 || newX >= cols || newZ < 0 || newZ >= rows){
				continue;}
			if (d % 2 == 0){
				if(map[newX][newZ] || map[nd.X][newZ] || map[newX][nd.Z]){
					continue;
				}
				g = 141;
			}
			else if(map[newX][newZ])
			{
				continue;
			}
			else
				g = 100;
			bool inClosed = false;
			for	(int i = 0; i < closedCount; i++)
			{
				if (newX == closed[i].X && newZ == closed[i].Z)
				{
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
						openList[i].Previous = closedCount-1;
						openList[i].G = nd.G + g;
						openList[i].F = nd.G + g + H(openList[i].X, openList[i].Z);
						openList[i].Dir = d;
					}
					break;
				}
			}
			if (!inOpen)
			{
			openList[openCount] = AStarNode(nd.X + moveX[d], nd.Z + moveZ[d], closedCount - 1, nd.G + g, nd.G + g + H(nd.X + moveX[d], nd.Z + moveZ[d]), d);
			openCount++;
			}
		}
	}
	//explorarea pentru nodul de start
	void FirstExplore(){
		short int moveX [] = {1, 1, 1, 0, -1, -1, -1, 0};
		short int moveZ [] = {-1, 0, 1, 1, 1, 0, -1, -1};
		AStarNode nd = closed[closedCount-1];
		for	(int d = 0; d < 8; d++){
			unsigned short int g;
			if (nd.X + moveX[d] < 0 || nd.X + moveX[d] >= cols || nd.Z + moveZ[d] < 0 || nd.Z + moveZ[d] >= rows){
				continue;}
			if (d % 2 == 0){
				if(map[nd.X + moveX[d]][nd.Z + moveZ[d]] || map[nd.X][nd.Z + moveZ[d]] || map[nd.X + moveX[d]][nd.Z]){
					continue;
				}
				g = 141; // miscari pe diagonala
			}
			else if(map[nd.X + moveX[d]][nd.Z + moveZ[d]])
			{
				continue;
			}
			else
				g = 100; // miscari pe verticala/orizontala
			openList[openCount] = AStarNode(nd.X + moveX[d], nd.Z + moveZ[d], closedCount - 1, g, g + H(nd.X + moveX[d], nd.Z + moveZ[d]), d);
			openCount++;
		}
	}
	
	// functia pentru crearea drumului cel mai scurt
	void BuildPath(AStarNode target)
	{
		AStarNode asn = target;
		while (asn.X != meX || asn.Z != meZ)
		{
			goldenPathX[pathCount] = asn.X;
			goldenPathZ[pathCount] = asn.Z;
			orientation[pathCount] = asn.Dir;
			std::cout<<asn.X<<","<<asn.Z<<" ; "<<asn.Dir<<" ;F "<<asn.F<<" ;G "<<asn.G<<" ;H "<<H(asn.X, asn.Z)<<std::endl; //afisarea coordonatelor prin care urmeaza sa treaca robotul si functia F si G
			pathCount++;
			asn = closed[asn.Previous];
		}
	}
	public:
		unsigned short int pathCount;
		unsigned short int goldenPathX[830];
		unsigned short int goldenPathZ[830];
		unsigned short int orientation[830];
		Pathfinding(bool **B, unsigned short int startX, unsigned short int startZ, unsigned short int endX, unsigned short int endZ, unsigned short int dx, unsigned short int dz){
			std::cout<<"start x, z "<<startX<<", "<<startZ<<", tinta x, z "<<endX<<", "<<endZ<<std::endl;
			map = B;
			meX = startX;
			meZ = startZ;
			targetX = endX;
			targetZ = endZ;
			openCount = 0, closedCount = 0, pathCount = 0;
			cols = dx;
			rows = dz;
		}
		//metoda care cauta un cel mai scurt drum catre tinta si il salveaza in campurile publice goldenPathX, goldenPathZ si orientation. intoare false daca nu se gaseste un drum
		bool Astar(){
			bool fe = true;
			AStarNode current;
			unsigned short int crtNr;
			openList[openCount] = AStarNode(meX, meZ, 1199, 0, H(meX, meZ), 7);
			openCount++;
			while (openCount > 0){
				crtNr = Promising(openCount);
				current = openList[crtNr];
				if (current.X == targetX && current.Z == targetZ){
					std::cout<<"Am gasit o cale pana la tinta!"<<std::endl;
					BuildPath(current);
					return true;
				}
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