#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;

template <typename Type>
inline Type max(Type t1, Type t2)
{
	return t1 > t2 ? t1 : t2;
}

template <typename elemType>
inline elemType max(const vector<elemType> &vec)
{
	return *max_element(vec.begin(), vec.end());
}

template <typename arrayType>
inline arrayType max(const arrayType* parray, int size)
{
	return *max_element(parray, parray + size);
}

int main()
{
	string sarray[] = {"WE", "HERO", "COUNTRY", "of", "I"};
	vector<string> svec(sarray, sarray + 5);
	
	int iarray[] = {12, 43, 54, 65, 897};
	vector<int> ivec(iarray, iarray + 5);
	
	float farray[] = {32.3, 43.3, 54, 65};
	vector<float> fvec(farray, farray + 4);
	
	int a1 = max(ivec);
	int a2 = max(iarray, 5);
	cout << a1;
	int imax = max(a1,a2);//max(max(ivec), max(iarray, 5));
	/*float fmax = max(max(fvec), max(farray, 4));
	string smax = max(max(svec), max(sarray, 5));
	
	cout << "imax should be 897 -- found: " << imax << '\n'
		 << "fmax should be 65 -- found: " << fmax << '\n'
		 << "smax should be of -- found: " << smax << endl;*/
}
