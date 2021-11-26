class Solution {
public:
    vector<int> intersection(vector<int>& nums1, vector<int>& nums2) {
        vector<int> res;
        set<int> set1, resSet;
        
        
        for (int& num : nums1)
            set1.insert(num);
        for (int& num : nums2) {
            if (set1.find(num) != set1.end())
                resSet.insert(num);
        }
        
        for (auto it = resSet.begin(); it != resSet.end(); it++)
            res.push_back(*it);
        
        return res;
    }
};
