#include <iostream>
#include <string>

using namespace std;

unsigned int Fibonacci1(int n);
unsigned int Fibonacci2(int n);
unsigned int Factorial(int n);
void Tower_of_Hanoi(int n, string A, string B, string C);

int main()
{  int n = 1;
   unsigned int res = 1;
   do 
   { cout << "Please input n:";
     cin >> n;
     if (n < 1)
     cout << "Invalid input. n shoule be >= 1." << endl;
   } while (n < 1);
   res = Fibonacci2(n);
   cout << "f(n)=" << res << endl;
   res = Fibonacci1(n);
   cout << "f(n)=" << res << endl;
   res = Factorial(n);
   cout << n <<"!=" << res << endl;
   
   Tower_of_Hanoi(3, "Column1", "Column2", "Column3");  // Column1 is the original column
}                                                       // Column2 is the intermediate column
                                                        // Column3 is the target column
unsigned int Fibonacci1(int n)    // Loop to compute f(n)
{  unsigned int f1=1, f2=1, f3, i=3;
    if (n == 1 || n == 2)
     return 1;
   for(; i<=n; i++)
   {  f3 = f1 + f2;
      f1 = f2;
      f2 = f3;
   }
   return f3;
}

unsigned int Fibonacci2(int n)               // These functions are very classical examples for recurision.
{  if (n == 1 || n ==2 )
     return 1;
   else
     return Fibonacci2(n-1)+Fibonacci2(n-2);
}

unsigned int Factorial(int n)
{  if (n == 1)
     return 1;
   else
     return n*Factorial(n-1);
}

void Tower_of_Hanoi(int n, string A, string B, string C)
{ 
   if (n == 1)
   cout << "Move from " << A  <<" to "  << C  <<endl;
   else
   {  Tower_of_Hanoi(n-1, A, C, B);
      Tower_of_Hanoi(1, A, B, C);
      Tower_of_Hanoi(n-1, B, A, C);
   }
 } 
