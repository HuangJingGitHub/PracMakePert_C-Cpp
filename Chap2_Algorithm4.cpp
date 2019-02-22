/**
* Linear-time maximum contiguous subsequence sum algorithm.
*/
#include <iostream>
#include <vector>

int maxSubSum4( const vector<int> &a)
{
	int maxSum = 0, thisSum = 0; 
	for (int j = 0; j < a.size(); ++j)
	{
		thisSum += a[j];
		
		if (thisSum > maxSum)
			maxSum = thisSum;
		else if (thisSum < 0)
			thisSum = 0;
	}
	return maxSum;
}

/*Binary Search*/

template <typename Comparable>
int binarySearch( const vector<Comparable> &a, const Comparable &x)
{
	int low = 0, high = a.size() - 1, NOT_FOUND = -1;
	
	while (low <= high)
	{
		int mid = (low + high) / 2;
		
		if ( a[mid] < x)
			low = mid + 1;
		else if 
			high = mid - 1;
		else
			return mid;
	}
	return NOT_FOUND;
 } 

/* Euclid's Algorithm */
long long gcd(long long m, long long n)
{
	while (n != 0)
	{
		long long rem = m % n;
		m = n;
		n =rem;
	}
	return m;
}
 
