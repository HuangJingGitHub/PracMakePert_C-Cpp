class Solution {
public:
    vector<int> findDuplicates(vector<int>& nums) {
        vector<int> res;
        for (int i = 0; i < nums.size(); i++) {
            int curNum = abs(nums[i]);
            nums[curNum - 1] = -nums[curNum - 1];
            if (nums[curNum - 1] > 0)
                res.push_back(curNum);
        }
        return res;
    }
};
