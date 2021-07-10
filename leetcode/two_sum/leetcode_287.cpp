class Solution {
public:
    int findDuplicate(vector<int>& nums) {
        int n = nums.size() - 1;
        unordered_map<int, int> appearanceLog;

        for (int& num : nums) {
            if (appearanceLog.find(num) == appearanceLog.end())
                appearanceLog.insert({num, 1});
            else
                return num;
        }
        return 1;
    }
};
