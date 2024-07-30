class Solution {
public:
    int findLHS(vector<int>& nums) {
        int res = 0;
        unordered_map<int, int> frequency;
        for (int& num : nums)
            frequency[num]++;

        for (auto it = frequency.begin(); it != frequency.end(); it++) {
            if (frequency.count(it->first + 1) != 0)
                res = max(res, it->second + frequency[it->first + 1]);
            if (frequency.count(it->first - 1) != 0)
                res = max(res, it->second + frequency[it->first - 1]);                
        }
        return res;
    }
};
