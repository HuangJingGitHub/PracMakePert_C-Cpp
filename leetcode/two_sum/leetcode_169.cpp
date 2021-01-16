// common solution 
class Solution {
public:
    int majorityElement(vector<int>& nums) {
        int res, frequency;
        unordered_map<int, int> frequencyMap;
        for (int x : nums)
            frequencyMap[x]++;
        
        auto itr = frequencyMap.begin();
        res = itr->first;
        frequency = itr->second;
        for (; itr != frequencyMap.end(); itr++)
            if (itr->second > frequency) {
                frequency = itr->second;
                res = itr->first;
            }
        return res;
    }
};


// Moore vote method, O(1) space complexity
class Solution {
public:
    int majorityElement(vector<int>& nums) {
        int res = nums[0], cnt = 1;
        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] == res)
                cnt++;
            else {
                cnt--;
                if (cnt == 0) {
                    res = nums[i];
                    cnt = 1;
                }
            }
        }
        return res;
    }
};
