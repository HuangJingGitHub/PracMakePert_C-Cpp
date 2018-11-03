#include <iostream>
#include "GlobalWrapper.h"
#define pi 3.1415926
using namespace std;

double mysin(double x);
double mycos(double x);

int main()
{
	globalWrapper gw;
	string ss = gw.program_name();
	cout << gw.program_name() << endl;
	//_program_name = "Exercise4_3";
	cout << "sin(pi/4) = " << mysin(pi / 4) << endl;
}

double mysin(double x)
{
	if (-0.005 < x && x < 0.005)
		return x - x*x*x / 6;
	else
		return 2 * mysin(x / 2) * mycos(x / 2);
}

double mycos(double x)
{
	if (-0.005 < x && x < 0.005)
		return 1 - x*x / 2;
	else
		return 1 - 2 * mysin(x / 2)*mysin(x / 2);
}