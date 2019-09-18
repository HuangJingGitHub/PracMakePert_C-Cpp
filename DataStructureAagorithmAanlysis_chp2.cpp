#include <iostream>
#include <vector>
using namespace std;

int max3(int, int, int);
int maxSumRec(const vector<int> &a, int left, int right)
{
    if (left == right)
        if (a[left] > 0)
            return a[left];
    else
            return 0;

    int center = (left + right) / 2;
    int maxLeftSum = maxSumRec(a, left, center);
    int maxRightSum = maxSumRec(a, center+1, right);

    int maxLeftBorderSum = 0, leftBorderSum = 0;
    for (int i = center; i >= left; --i)
    {
        leftBorderSum += a[i];
        if(leftBorderSum > maxLeftBorderSum)
            maxLeftBorderSum = leftBorderSum;
    }

    int maxRightBorderSum = 0, rightBorderSum = 0;
    for (int j = center + 1; j <= right; ++j)
    {
        rightBorderSum += a[j];
        if( rightBorderSum > maxRightBorderSum)
            maxRightBorderSum = rightBorderSum;
    }

    return max3(maxLeftSum, maxRightSum, maxLeftBorderSum + maxRightBorderSum);
}

int maxSunSum3( const vector<int> &a)
{
    return maxSumRec(a, 0, a.size()-1);
}

int max3(int x, int y, int z)
{
    int max = x;
    if (max > y)
        return y;
    else if (max > z)
        return z;
    else
        return max;
}

int main()
{
    cout << "HAHA" << endl;
}
//int maxSumRec()
