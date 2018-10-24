#include <iostream>
using namespace std;

int main()
{
	const int array_size = 100;
	int ia[array_size];
	int ival, icnt = 0, sum = 0, average = 0;
	
	while (cin >> ival && icnt < array_size)
		ia[icnt++] = ival;
	
	for (int ix = 0; ix < icnt; ++ix)
		sum += ia[ix];
	
	if(icnt != 0)
	{
		 average = sum / icnt;
	 	cout << "Sum of " << icnt
			 << " elements: " << sum
		 	 << ".Average: "  << average << endl;
	}
	else
	cout << "Empty array!" << endl;
}
