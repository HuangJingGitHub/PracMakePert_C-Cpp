#include <iostream>
#include "num_sequence.h"
#include "num_sequence.cpp"

using namespace std;

int main()
{
	Fibonacci fib;
	cout << "fib: beginning at element 1 for 1 element: "
		 << fib << endl;
	
	Fibonacci fib2(16);
	cout << "fib2: beginning at element 1 for 16 elements:"
	     << fib2 << endl;
    
    Fibonacci fib3(8,12);
    cout << "fib3: beginning at element 12 for 8 elements"
         << fib3 << endl;
 } 
