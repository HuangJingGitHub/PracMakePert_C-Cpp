#include <iostream>
#include "random.h"
using namespace std;

int main()
{  int i, t1;
   double t2;
   
   Randomize();
   for( i = 1; i <= 8; i++)
   {   t1 = GenerateRandomNumber(1,99);
       cout << t1 << " ";
   }
   cout << '\n';
   
   for(i = 1; i <= 8; i++)
   {   t2 = GenerateRandomReal(1.0, 99.0);
       cout << t2 << " ";
   }
   cout << '\n';
   
   for(i = 1; i <= 8; i++)
   {   t2 = GenerateRandomStd();
       cout << t2 << " ";
   }
   cout << '\n';
}


