class Solution {
public:
    bool makesquare(vector<int>& matchsticks) {
        int sum = 0;
        for (int& element : matchsticks)
            sum += element;
        if (sum % 4 != 0)
            return false;
        
        sort(matchsticks.begin(), matchsticks.end(), greater<int>());
        vector<int> side_length(4, 0);
        return backtrack(matchsticks, 0, sum >> 2, side_length);
    }

    bool backtrack(vector<int>& nums, int idx, int target, vector<int>& side_length) {
        if (idx == nums.size()) {
            if (side_length[0] == side_length[1] && side_length[1] == side_length[2] && side_length[2] == side_length[3])
                return true;
            return false;
        }

        for (int i = 0; i < 4; i++) {
            if (side_length[i] + nums[idx] > target || (i > 0 && side_length[i] == side_length[i - 1]) || (idx == 0 && i != 0)) 
                continue;
            
            side_length[i] += nums[idx];
            if (backtrack(nums, idx + 1, target, side_length))
                return true;
            side_length[i] -= nums[idx];
        }
        return false;
    }
};
