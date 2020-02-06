class Solution {
public:
    vector<vector<int>> permute(vector<int>& nums) {

        vector<vector<int>> res;
        if (nums.size() == 1 || nums.size() == 0){
            res.push_back(nums);
            return res;
        }

        vector<int> initVec(nums[0]);
        res.push_back(initVec);
        for(int i = 1; i < nums.size(); i++){
            for (int j = 0, currentSize = res.size(); j <= i; j++){
                for (int k = 0; k < currentSize; k++){
                    cout << res.size() << " ";
                    vector<int> x = res[k];
                    x.insert(x.begin() + j, nums[i]);
                    res.push_back(x);
                }
            }
        }
        return res;
    }
};
