class Solution {
public:
    int pivotIndex(vector<int>& nums) {
        int n = nums.size();
        vector<int> leftSum(n, 0), rightSum(n, 0);
        for (int i = 1; i < n; i++){
            leftSum[i] = leftSum[i - 1] + nums[i - 1];
            rightSum[n - 1 - i] = rightSum[n - i] + nums[n - i];
        }
        for (int i = 0; i < n; i++)
            if (leftSum[i] == rightSum[i])
                return i;
        return -1;
    }
};

// much less memory cost
class Solution {
public:
    int pivotIndex(vector<int>& nums) {
        int sum = 0;
        for (int i = 0; i < nums.size(); i++)
            sum += nums[i];
        for (int i = 0, leftSum = 0; i < nums.size(); i++){
            if (leftSum * 2 + nums[i] == sum)
                return i;
            leftSum += nums[i];
        }
        return -1;
    }
};
