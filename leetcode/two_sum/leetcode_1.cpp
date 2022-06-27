class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        vector<int> res;
        unordered_map<int, int> numToIdx;
        
        for (int i = 0; i < nums.size(); i++){
            if (numToIdx.find(target - nums[i]) != numToIdx.end()){
                res.push_back(numToIdx.find(target - nums[i])->second);
                res.push_back(i);
                return res;
            }
            numToIdx.insert(std::make_pair(nums[i], i));
        }
        return res;
    }
};
