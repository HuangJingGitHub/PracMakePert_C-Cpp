// Chapter3.5 Using the Generic Alogrithms
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>

using namespace std;

// Jadge whether an element is in the vector
bool is_elem(vector<int> &vec, int elem)
{
	int max_value = *max_element(vec.begin(), vec.end());
	if (max_value < elem)
		return false;
	if (max_value == elem)
		return true;
	
	vector<int> temp(vec.size());
	copy(vec.begin(), vec.end(), temp.begin());
	sort(temp.begin(), temp.end());    // Guatantee the vector is in ascending order for binary_search()
	return binary_search(temp.begin(), temp.end(), elem);
}

vector<int> filter1(const vector<int> &vec, int filter_value, bool (*pred)(int, int))
{                                                  // *pred is a pointer to function.
	vector<int> nvec;                              // This function gets the elements satisfing
	for (int ix=0; ix < vec.size(); ++ix)          // certain condition, like > 10
		if (pred(vec[ix], filter_value))
			nvec.push_back(vec[ix]);
	return nvec;
}

bool less_than(int v1, int v2)
{
	return v1 < v2 ? true : false;
 } 
 
 bool less_equal(int v1, int v2)
 {
 	return v1 <= v2 ? true : false;
  } 
 
 bool greater_than(int v1, int v2)
 {
 	return v1 > v2 ? true : false;
 }
 
 bool greater_equal(int v1, int v2)
 {
 	return v1 >= v2 ? true : false;
 }

// To count how many times a value occurs in a given vector.
int count_occurts(const vector<int> &vec, int val)
{
	vector<int>::const_iterator iter = vec.begin();
	int occurs_count = 0;
	while ((iter = find(iter, vec.end(), val)) != vec.end())
	{
		++occurs_count;
		++iter;
	}
	return occurs_count;
}

vector<int> filter2(const vector<int> &vec, int val, less<int> &lt)
{
	vector<int> nvec;
	vector<int>::const_iterator iter = vec.begin();
	while ((iter = find_if(iter, vec.end(),bind2nd(lt, val))) != vec.end())
	{                                      // Use function objects and binder
		nvec.push_back(*iter);
		iter++;
	}
	return nvec;
}

template <typename InputIterator, typename OutputIterator, typename ElemType, typename Comp>
OutputIterator filter3(InputIterator first, InputIterator last, 
                       OutputIterator at, const ElemType &val, Comp pred)
{
	while ((first = find_if(first, last, bind2nd(pred, val))) != last)
	{
		*at++ = *first++;  //*at = *first; at++ and first++;
	}
	return at;
}
int main()
{
	int a[8] = {1,2,4,3,5,7,6,8};
	vector<int> vec(a, a+8);
	bool tof = is_elem(vec, 8);
	cout << tof << endl;
}



