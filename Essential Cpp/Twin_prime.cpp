#include <iostream>
#include <cmath>
#include <bits.h>
#include <string>
using namespace std;

int main()
{
	const int num_primes = 10005;
	bool primes[num_primes];
	for (int i = 2; i != num_primes; i++)
		primes[i] = true;

	for (int i = 2; i != int(sqrt(num_primes)); ++i)
	{
		if (primes[i])
		{
			for (int j = 2; i * j < num_primes; ++j)
				primes[i * j] = false;
		}
	}

	int n;
	cout << "Input an integer:\n";
	cin >> n;
	while (1)
	{
		if (n <= 2 || n > 10005)
		{
			cout << "INVALID INPUT! n should be in [3, 10005]. Please input again: \n";
			cin >> n;
		}
		else
			break;
	}

	cout << "Twin primes are: \n";
	for (int i = n; i - 2 >= 0; --i)
		if (primes[i] && primes[i - 2])
		{
			cout << i - 2 << ' ' << i << endl;
			break;
		}

	cout << endl;
	cin.ignore();  // getline
	string str, repl = "cat";
	getline(cin, str);
	cout << "Original text: " << str << endl;
	for (int j = 0; j < (int)str.size(); j++)
	{
		string key = str.substr(j, 3);
		if (key == "dog")
		for (int k = 0; k < 3; k++)
			str[j + k] = repl[k];
	}
	cout << "The replaced string is: " << str << endl;
	return 0;
}