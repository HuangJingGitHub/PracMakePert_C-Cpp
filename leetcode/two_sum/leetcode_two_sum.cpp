#include <iostream>
#include <unordered_map>
#include <vector>
#include <typeinfo>

using namespace std;

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
            if (range.first != umap.end() )
            {
            	cout << "subTarget = " << subTarget << endl;
            	if (range.first != umap.find(nums[i]))
            	{
            		vector<int> index{i, range.first->second};
            		return index;
				}
            	else if (range.first == umap.find(nums[i]) && next(range.first, 1) != umap.end())
                {
                	for (auto itr = range.first; itr != range.second; itr++)
                		cout << itr->first << " " << itr->second << endl;
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
	vector<int> testVec{3,2,4,4,5,3,8,3}, ans;
	int testTarget = 6;
	Solution sol;
	ans = sol.twoSum(testVec, testTarget);
	
	for (int x : ans)
		cout << x << " ";
}


