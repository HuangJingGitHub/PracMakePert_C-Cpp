class Solution {
public:
    vector<vector<int>> fourSum(vector<int>& nums, int target) {
        sort(nums.begin(), nums.end());
        vector<vector<int>> ans;

        if (nums.size() < 4)
            return ans;
        
        for (int i = 0; i < nums.size() - 3; i++)
        {
            if (i > 0 && nums[i] == nums[i-1])
                continue;
            for (int j = i + 1; j < nums.size() - 2; j++)
            {
                if (nums[i] + nums[nums.size() - 3] + nums[nums.size() - 2] + nums[nums.size() - 1] < target)
                   break;

                if (j > 1 && nums[j] == nums[j-1] && j != i + 1)
                    continue;
                int threeSumTarget = target - nums[i];
                int leftPointer = j + 1, rightPointer = nums.size() - 1;

                while (leftPointer < rightPointer)
                {
                    int sum = nums[j] + nums[leftPointer] + nums[rightPointer];
                    if (sum < threeSumTarget)
                        leftPointer++;
                    else if (sum > threeSumTarget)
                        rightPointer--;
                    else
                    {
                        vector<int> quadruplet{nums[i], nums[j], nums[leftPointer], nums[rightPointer]};
                        ans.push_back(quadruplet);
                        leftPointer++;
                        rightPointer--;
                        while(leftPointer < rightPointer && nums[leftPointer] == nums[leftPointer - 1])
                            leftPointer++;
                        while(leftPointer < rightPointer && nums[rightPointer] == nums[rightPointer + 1])
                            rightPointer--;
                    }
                }

            }
        }
        return ans;
    }
};
