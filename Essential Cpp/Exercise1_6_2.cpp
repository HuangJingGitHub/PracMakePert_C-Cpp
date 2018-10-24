#include <iostream>
#include <vector>
using namespace std;
int main()
{
	vector<int> ivec;
	int ival, sum = 0;
	while(cin >> ival )
	{
		ivec.push_back(ival);
	}
	
	for (int ix = 0; ix < ivec.size(); ++ix)
		sum += ivec[ix];
	
	int average = sum / ivec.size();
	cout << "The average is " << average << endl;
 } 
