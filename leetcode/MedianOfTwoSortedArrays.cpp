#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

class Solution {
public:
	double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
		nums1.insert(nums1.end(), nums2.begin(), nums2.end());
		sort(nums1.begin(), nums1.end());
		//nums1.erase(unique(nums1.begin(), nums1.end()), nums1.end());
		//for (int i = 0; i < nums1.size(); i++) cout << nums1[i] << " ";
		if (nums1.size() % 2 == 0)
			return (nums1[nums1.size() / 2] + nums1[nums1.size() / 2 - 1]) / 2.0; // 2.0 not 2 (avoid int arithmetic
		else
			return nums1[(nums1.size() - 1) / 2];
	}
};

class Solution_standard
{
public:
	double findMdeianSortedArrays(vector<int>& nums1, vector<int> &nums2)
	{
		int n = nums1.size();
		int m = nums2.size();

		if (n > m)
		{
			return findMdeianSortedArrays(nums2, nums1);
		}
		int LMax1, LMax2, RMin1, RMin2, c1, c2, lo = 0, hi = 2 * n;
		while (lo <= hi)
		{
			c1 = (lo + hi) / 2;
			c2 = m + n - c1;

			LMax1 = (c1 == 0) ? INT_MIN : nums1[(c1 - 1) / 2];
			RMin1 = (c1 == 2 * n) ? INT_MAX : nums1[c1 / 2];
			LMax2 = (c2 == 0) ? INT_MIN : nums2[(c2 - 1) / 2];
			RMin2 = (c2 == 2 * m) ? INT_MAX : nums2[c2 / 2];

			if (LMax1 > RMin2)
				hi = c1 - 1;
			else if (LMax2 > RMin1)
				lo = c1 + 1;
			else
				break;
		}
		return (max(LMax1, LMax2) + min(RMin1, RMin2)) / 2.0;
	}
};

class Solution3 {
public:
	string longestPalindrome(string s) {
		vector<int> indexStore;
		int maxLen = 1, currentLen = 1;
		string currentStr;
		for (int i = 1; i < s.size(); i++)
			if (s[i] == s[i + 1] || s[i - 1] == s[i + 1])\
				indexStore.push_back(i);
		for (int i = 0; i < indexStore.size(); i++)
			currentStr = (palindromeStr(s, indexStore[i]).size() > currentStr.size()) ? palindromeStr(s, indexStore[i]) : currentStr;
		return currentStr;

	}
	string palindromeStr(string s, int position)
	{
		int left, right;
		if (s[position] == s[position + 1])
		{
			left = position - 1, right = position + 2;
		}
		else
		{
			left = position - 2, right = position + 2;
		}
	
		for (; (left != 0) && (right != s.size() - 1); left--, right++)
		{
			if (s[left] != s[right])
			{
				return s.substr(left + 1, right - left - 1);
			}
		}
	}
};

class Solution3_1 {
public:
	string longestPalindrome(string s) {
		vector<int> indexStore;
		int maxLen = 1, currentLen = 1;
		string currentStr;
		for (int i = 1; i < s.size(); i++)
			if (s[i] == s[i + 1] || s[i - 1] == s[i + 1])\
				indexStore.push_back(i);
		for (int i = 0; i < indexStore.size(); i++)
			currentStr = (palindromeStr(s, indexStore[i]).size() > currentStr.size()) ? palindromeStr(s, indexStore[i]) : currentStr;
		return currentStr;

	}
	string palindromeStr(string s, int position)
	{
		int left, right;
		string temp;
		if (s[position] == s[position + 1])
		{
			left = position - 1, right = position + 2;
		}
		else
		{
			left = position - 2, right = position + 2;
		}

		for (; (left != 0) && (right != s.size() - 1); left--, right++)
			if (s[left] != s[right])
				temp = s.substr(left + 1, right - left - 1);

		return temp;
	}
};

int main()
{
	int a = 1;
}
