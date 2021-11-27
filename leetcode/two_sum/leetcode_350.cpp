class Solution {
public:
    vector<int> intersect(vector<int>& nums1, vector<int>& nums2) {
        vector<int> res;
        map<int, int> map1, map2;
        
        for (int& num : nums1)
            map1[num]++;
        for (int& num : nums2)
            map2[num]++;
        
        for (auto it = map1.begin(); it != map1.end(); it++) {
            int curNum = it->first, curOccurence = it->second;
            if (map2.find(curNum) != map2.end()) {
                curOccurence = min(curOccurence, map2[curNum]);
                for (int i = 0; i < curOccurence; i++)
                    res.push_back(curNum);
            }
        }

        return res;
    }
};
