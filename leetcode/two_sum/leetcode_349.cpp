class Solution {
public:
    vector<int> intersection(vector<int>& nums1, vector<int>& nums2) {
        vector<int> res;
        map<int, int> log1, log2, totalLog;
        
        for (int& num : nums1) {
            log1[num]++;
            totalLog[num]++;
        }
        for (int& num : nums2) {
            log2[num]++;
            totalLog[num]++;
        }
        
        for (auto it = totalLog(); it != totalLog.end(); it++)
            if (it->second > 1)å
                
    }
};
