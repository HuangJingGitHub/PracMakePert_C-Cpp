#include <iostream>
using namespace std;

void swap(int &a, int &b)
{  int temp;
   temp = a;
   a = b;
   b = temp;
}

bool print_sequence(int pos)
{  if (pos <= 0 || pos > 1024)
   {  cerr << "Invalid position: " << pos
           << "--can't handle request!\n";
           return false;
   }
   cout << "The Fibonacci Sequence for "
        << pos  << " position: \n\t";
   switch (pos)
   { default:
   	 case 2:
   	 	cout << "1 ";
   	 	// No break
   	 case 1:
   	 	cout << "1 ";
   	 	break;
   }
   
   int elem;
   int n_2 = 1, n_1 = 1;
   for (int ix = 3; ix <= pos; ++ix)
   {  elem = n_2 + n_1;
      n_2 = n_1;
      n_1 = elem;
      cout << elem
           << (!(ix % 10) ? "\n\t" : " ");  //print 10 elements to a line
	}
	cout << endl;   
}

int main()
{  int v1 = 1, v2 = 2;
   swap(v1, v2);
   cout << v1 << ' ' << v2 << '\n';
   print_sequence(20);
   
}
