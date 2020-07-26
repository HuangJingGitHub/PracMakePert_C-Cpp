class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        vector<int> res;
        unordered_map<int, int> hashMap;
        for (int i = 0; i < nums.size(); i++){
            if (hashMap.find(nums[i]) != hashMap.end()){
                res.push_back(hashMap.find(nums[i])->second);
                res.push_back(i);
                return res;
            }
            hashMap.insert(std::make_pair(target-nums[i], i));  // Instead store the pair (nums[i], i), store (target - nums[i], i)
        }
        return res;
    }
};
