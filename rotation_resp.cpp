#include <iostream>
#include <cmath>
#include <iomanip>

#define pi 3.1415926
using namespace std;

double* axis_ang(double alp, double bet, double gam)
{
	double R1[3][3] = {0}, R2[3][3] = {0},
	       R3[3][3] = {0}, T[3][3] = {0}, R[3][3] = {0};
	R1[0][0] = cos(alp);
	R1[0][1] = -1*sin(alp);
	R1[1][0] = sin(alp);
	R1[1][1] = cos(alp);
	R1[2][2] = 1;
	
	R2[2][2] = cos(bet);
	R2[2][0] = -1*sin(bet);
	R2[0][2] = sin(bet);
	R2[0][0] = cos(bet);
	R2[1][1] = 1;	
	
	R3[0][0] = cos(gam);
	R3[0][1] = -1*sin(gam);
	R3[1][0] = sin(gam);
	R3[1][1] = cos(gam);
	R3[2][2] = 1;
	
	for(int r=0; r<3; r++)
		for(int c=0; c<3; c++)
			for(int i=0; i<3; i++)
				T[r][c] +=  R1[r][i]*R2[i][c];
				
	for(int r=0; r<3; r++)
		for(int c=0; c<3; c++)
			for(int i=0; i<3; i++)
				R[r][c] += T[r][i]*R3[i][c];
				
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			cout << setprecision(3) 
			     << setiosflags(ios::fixed)
			     << R[i][j] << " ";
			if (j==2)
			cout << endl;
		}
	return R[0];
}

int main()
{
	double a = pi/4, b = 0, c = 0;
	double* Rot;
	Rot = axis_ang(a,b,c);
	//for(int i=0; i<3; i++)
	//cout << Rot[i] << " ";
}
