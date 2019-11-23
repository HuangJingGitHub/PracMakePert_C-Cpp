#include <iostream>
#include <unordered_map>
#include <vector>
using namespace std;

bool isMatch(string text, string pattern)
{
	int i = 0, j = 0;
	while (j < pattern.size())
	{
		if (i >= text.size())
			return false;
		if (pattern[j++] != text[i++])
			return false;
	}
	return j == text.size();
}

class Solution {
public: 
    vector<int> twoSum(vector<int>& nums, int target)
    {
        std::unordered_multimap<int, int> umap;
        for (int i = 0; i < nums.size() ; i++)
        {
            umap.insert(std::make_pair(nums[i], i));
        }
        
        for (int i = 0, subTarget; i < nums.size() - 1; i++)
        {
            subTarget = target - nums[i];
            auto range = umap.equal_range(subTarget);
            if (range.first != umap.end())
            {
                if (range.first != umap.find(nums[i]))
                {
                    vector<int> index{i, range.first->second};
                    return index;
                }
                else if (std::next(range.first, 1) != umap.end())
                {
                    vector<int> index{i, range.first->second};
                    return index;
                }
            }
            
        }
        
        vector<int> noAnswer;
        return noAnswer;
    }
};

int main()
{
	vector<int> testVec{3,3,2,4}, ans;
	int testTarget = 6;
	Solution sol;
	ans = sol.twoSum(testVec, testTarget);
	
	for (int x : ans)
		cout << x << " ";
}





