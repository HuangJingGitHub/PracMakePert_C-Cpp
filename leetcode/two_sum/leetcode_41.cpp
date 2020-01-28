class Solution {
public:
    int firstMissingPositive(vector<int>& nums) {
        int smallestPositiveInt = pow(2, 32);
        for (int i = 0; i < nums.size(); i++){
            if (nums[i] <= 0)
                continue;
            if (nums[i] == smallestPositiveInt + 1)  

        }
        
    }
};
