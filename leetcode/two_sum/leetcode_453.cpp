class Solution {
public:
    int minMoves(vector<int>& nums) {
        long long sum = 0;
        int min_num = nums[0];

        for (int& num : nums) {
            sum += num;
            min_num = min(min_num, num);
        }

        return sum - nums.size() * min_num;
    }
};
