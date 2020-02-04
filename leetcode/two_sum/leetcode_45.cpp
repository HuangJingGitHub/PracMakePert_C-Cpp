class Solution {
public:
    int jump(vector<int>& nums) {
        int jumpSum = 0, currentPos, nextPos, maxJump;
        for (currentPos = 0; currentPos + nums[currentPos] < nums.size(); ){
            maxJump = nums[currentPos];
            jumpSum++;
            for (int i = 2, nextPos = currentPos + 1; i <= maxJump && currentPos + i < nums.size(); i++){
                nextPos = (i + nums[currentPos + i]) > (nextPos - currentPos + nums[nextPos]) ? (currentPos + i) : nextPos;
                if ((nextPos + nums[nextPos]) >= nums.size())
                    return jumpSum;
            }
            currentPos = nextPos;
        }
        return jumpSum;
    }
};
