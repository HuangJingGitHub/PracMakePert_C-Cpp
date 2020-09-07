/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <vector>
using namespace std;

vector<int> merge(vector<int>, vector<int>);

void insertSort(vector<int>& A){
    for (int i = 1; i < A.size(); i++){
        int current = A[i];
        int j = i-1;
        while (j > -1 && A[j] > current){
            A[j+1] = A[j];
            A[j] = current;
            j--;
        }
    }
}

vector<int> mergeSort(vector<int> A){
    int len = A.size();
    vector<int> left, right;
    if (len <= 1)
        return A;
    else{
        left = mergeSort(vector<int>(A.begin(), A.begin() + len/2 + 1));
        right = mergeSort(vector<int>(A.begin() + len/2 + 1, A.end()));
    }
    
    return merge(left, right);
}

vector<int> merge(vector<int> left, vector<int> right){
    int lenL = left.size(), lenR = right.size(), 
        fingerL = 0, fingerR = 0;
    vector<int> res(lenL + lenR, 0);
    
    int i = 0;
    for ( ; i < res.size(); i++){
        if (fingerL < lenL && fingerR < lenR){
            if (left[fingerL] <= right[fingerR]){
                res[i] = left[fingerL];
                fingerL++;
            }
            else{
                res[i] = right[fingerR];
                fingerR++;
            }
        }
        else
            break;
    }
    while (fingerL < lenL){
        res[i] = left[fingerL];
        i++;
        fingerL++;
    }
    while (fingerR < lenR){
        res[i] = right[fingerR];
        i++;
        fingerR++;
    }
    
    return res;
}

int main()
{
    vector<int> v1{18,0, 32, -3, 32, 4, 2,546, 54};
    vector<int> sort_v1 = mergeSort(v1);
    // insertSort(test);
    
    for (auto x:sort_v1)
        cout << x << " ";

    return 0;
}
