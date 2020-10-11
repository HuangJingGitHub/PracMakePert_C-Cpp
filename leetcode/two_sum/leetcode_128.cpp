// interesting effective solution
class Solution {
public:
    int longestConsecutive(vector<int>& nums) {
        int res = 0;
        unordered_set<int> numsSet;
        for (int num : nums)
            numsSet.insert(num);
        
        for (int i = 0; i < nums.size(); i++){
            int curMax = 1;
            if (numsSet.find(nums[i] - 1) != numsSet.end())
                continue;
            else
                for (int j = 1; numsSet.find(nums[i] + j) != numsSet.end(); j++)
                    curMax++;
            
            res = max(res, curMax);
        }
        return res;
    }
};
