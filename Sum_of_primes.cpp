#include <iostream>
#include <vector>
using namespace std;

const int Ceiling_num = 10000;
const int Ceiling_sqrt = 100;

int main()
{
	bool is_prime[Ceiling_num];
	for (int ix = 0; ix < Ceiling_num; ++ix)
		is_prime[ix] = true;

	for (int ix = 2; ix < Ceiling_sqrt; ++ix)
		if (is_prime[ix - 1])
			for (int jx = 2; ix * jx <= Ceiling_num; ++jx)
				is_prime[ix * jx - 1] = false;

	vector<int> prime;
	for (int ix = 1; ix < Ceiling_num; ++ix)
		if (is_prime[ix])
			prime.push_back(ix + 1);
    
	int valid_num = prime.size(), sum = 0, n;
	cout << "Please input the number of primes: \n" << "n = ";
	cin >> n;
	while (true)
	{
		if (n > valid_num)
		{
			cout << "n exceeds the range! Please input again: \n" << "n = ";
			cin >> n;
		}
		if (n <= 0)
		{
			cout << "n should be a positive integer! Please input again: \n" << "n = ";
			cin >> n;
		}
		else
			break;
			
	}

	vector<int>::iterator st = prime.begin();
	for (int ix = 1; ix <= n; ++ix, ++st)
		sum += *st;

	cout << "The sum of the first " << n << (n == 1 ? " prime is " : " primes is ") << sum << ".\n\n";
}