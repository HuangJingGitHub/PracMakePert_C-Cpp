#include <iostream> 
#include <string>
#include <vector>
using namespace std;

template <typename elemType>
inline const elemType* begin(const vector<elemType> &vec)
{  return vec.empty() ? 0 : &vec[0];
}

template <typename elemType>
inline const elemType* end(const vector<elemType> &vec)
{  return vec.empty() ? 0 : &vec[0]+vec.size();
}

template <typename elemType>
const elemType* find1(const elemType *array, int size, const elemType &value)
{  if (!array || size < 1)
     return 0;
   for(int ix=0; ix<size; ++size, ++array)
     if (*array == value)
	   return array;
	return 0;  
 } 
 
 template <typename elemType>
 const elemType* find2(const elemType *first, const elemType *last, const elemType &value)
 {  if (!first || !last)
    return 0;
    for (; first != last; ++first)
      if (*first == value)
        return first;
    return 0;
  } 

 template <typename elemType>
 const elemType* find3(const vector<elemType> vec, const elemType &value)
 { const elemType *first = begin(vec);
   const elemType *last = end(vec);
 
   if (!first || !last)
    return 0;
    for (; first != last; ++first)
      if (*first == value)
        return first;
    return 0;
  } 

int main()
{  // Built-in array test:
   int    ia[8] = {1,2,4,3,2,3,5,5};
   double da[6] = {1.5, 3.3, 4, 7, 8, 6};
   string sa[4] = {"CHN","UK","US","CA"};
   string s1 = "CA";
   
   const int *p1 = find1(ia, 8, 5);
   cout << p1 <<'\n' << *p1  << '\n';
   const string *p2 = find1(sa, 4, s1);  // The invocation find1(sa, 4, "CA") is not valid, becaues
   cout << p2 <<'\n' << *p2;             // "CA" cannot be used as a reference semantics in function
                                         // find1(). So it need be defined before used.
   const int *p11 = find2(ia, ia+8, 5);
   cout << p11 <<'\n' << *p11 <<'\n';
   const string *p22 = find2(sa, sa+4, s1);
   cout << p22 << '\n' << *p22 <<'\n'; 
   
   // container class test
   const vector<int> va(ia, ia+8);
   const int *p13 = find3(va, 5);
   cout << p13 << '\n' << *p13 <<'\n'; 
   
}
