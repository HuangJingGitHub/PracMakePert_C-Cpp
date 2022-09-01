// greedy action
class Solution {
public:
    int jump(vector<int>& nums) {
        if (nums.size() == 1)
            return 0;

        int jumpSum = 1, currentPos, nextPos, maxJump;
        for (currentPos = 0; currentPos + nums[currentPos] < nums.size() - 1; currentPos = nextPos){
            maxJump = nums[currentPos];
            nextPos = currentPos + 1;
            for (int i = 2; i <= maxJump; i++) {
                nextPos = (currentPos + i + nums[currentPos + i]) > (nextPos + nums[nextPos]) ? (currentPos + i) : nextPos;
                if ((nextPos + nums[nextPos]) >= nums.size() - 1)
                    return ++jumpSum;
            }
            jumpSum++;
        }
        return jumpSum;
    }
};
