// interesting effective solution
class Solution {
public:
    int longestConsecutive(vector<int>& nums) {
        int res = 0;
        unordered_set<int> numsSet(nums.begin(), nums.end());
        
        for (int i = 0; i < nums.size(); i++) {
            if (numsSet.find(nums[i] - 1) != numsSet.end())
                continue;
            
            int curMax = 1;
            while (numSet.find(nums[i] + curMax != numSet.end())
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
                   
// straightforward and efficient
class Solution {
    int longestConsecutive(vector<int>& nums) {
        if (nums.empty() == true)
            return 0;
        
        int res = 1, curLongest = 1;
        sort(nums.begin(), nums.end());
        
        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] != nums[i - 1]) {
                if (nums[i] == nums[i - 1] + 1) {
                    curLongest++;
                }
                else {
                    res = max(res, curLongest);
                    curLongest = 1;
                }
            }
        }
        
        return max(res, curLongest);
    }
};                   
