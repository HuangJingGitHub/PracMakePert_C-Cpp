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


// dp solution, but it calls for sorting first.
class Solution {
public:
    int longestConsecutive(vector<int>& nums) {
        if (nums.size() == 0)
            return 0;
            
        int res = 1;
        map<int, int> numToRes;
        for (int& num : nums)
            numToRes[num] = 1;
        
        sort(nums.begin(), nums.end());
        for (int& num : nums) {
            if (numToRes.find(num - 1) != numToRes.end()) {
                numToRes[num] = numToRes[num - 1] + 1;
                res = max(res, numToRes[num]);
            }
        }

        return res;
    }
};
