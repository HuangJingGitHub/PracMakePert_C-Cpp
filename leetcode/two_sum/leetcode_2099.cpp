class Solution {
public:
    static bool sortBySecond(const pair<int, int>& a, const pair<int, int>& b) {
        return a.second < b.second;
    }

    vector<int> maxSubsequence(vector<int>& nums, int k) {
        vector<int> res(k, 0);
        vector<pair<int, int>> idxToVal;
        for (int i = 0; i < nums.size(); i++)
            idxToVal.push_back(pair<int, int>(i, nums[i]));
        sort(idxToVal.begin(), idxToVal.end(), sortBySecond);

        vector<int> idxVec;
        for (int i = idxToVal.size() - k; i < idxToVal.size(); i++)
            idxVec.push_back(idxToVal[i].first);
        sort(idxVec.begin(), idxVec.end());

        for (int i = 0; i < idxVec.size(); i++)
            res[i] = (nums[idxVec[i]]);

        return res;
    }
};
