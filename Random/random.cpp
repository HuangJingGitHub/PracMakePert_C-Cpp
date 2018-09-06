#include <iostream>
#include <cstdlib>
#include <ctime>
using namespace std;

void Randomize()
{  srand( (int)time(0) );
}

int GenerateRandomNumber(int low, int high)
{  double _d;
   if (low > high)
   {  cout << "GererateRandomNumber: Make sure the upper bound >= lower bound.";
      exit(1);
   }
  //  Randomize();
   _d = ((double)rand()) / (RAND_MAX + 1);
   return low + (int)(_d*(high - low));
 } 
 
double GenerateRandomReal(double low, double high)
{  double _d;
   if (low > high)
   {  cout << "GererateRandomReal: Make sure the upper bound >= lower bound.";
      exit(1);
   }
 //  Randomize();
   _d = ((double)rand()) / (RAND_MAX + 1);
   return low + _d*(high - low);
 } 
 
double GenerateRandomStd()
{  GenerateRandomReal(0.0 , 1.0);
}
