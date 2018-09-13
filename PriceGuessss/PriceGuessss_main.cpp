#include <iostream>
#include "random.h"
#include "guess.h"

using namespace std;
int main()
{   
	PrintWelcomeInfo();
	InitializeGame();
	double ratio = PlayGame();
	
	PrintGameOverInfo(ratio);
}
