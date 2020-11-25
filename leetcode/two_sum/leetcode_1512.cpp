class Solution {
public:
    int numIdenticalPairs(vector<int>& nums) {
        int res = 0;
        unordered_map<int, int> numTimes;
        for (auto x : nums)
            numTimes[x]++;
        
        for (auto itr = numTimes.begin(); itr != numTimes.end(); itr++)
            if (itr->second > 1)
                res += itr->second * (itr->second - 1) / 2;
        
        return res;
    }
};

// another method
class Solution {
public:
    int numIdenticalPairs(vector<int>& nums) {
        int res = 0;
        vector<int> numTimes(100, 0);
        
        for (int num : nums){
            res += numTimes[num - 1];  // pay attention to the index of num
            numTimes[num - 1]++;
        } 
        return res;
    }
};
