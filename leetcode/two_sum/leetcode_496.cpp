class Solution {
public:
    vector<int> nextGreaterElement(vector<int>& nums1, vector<int>& nums2) {
        vector<int> res(nums1.size(), 0);
        
        vector<int> nextIdx(nums2.size(), -1);
        for (int i = nums2.size() - 2; i >= 0; i--) {
            if (nums2[i] < nums2[i + 1])
                nextIdx[i] = i + 1;
            else {
                int j = i + 1;
                while (nextIdx[j] != -1) {
                    if (nums2[nextIdx[j]] > nums2[i]) {
                        nextIdx[i] = nextIdx[j];
                        break;
                    }
                    j = nextIdx[j];
                }
            }
        }

        unordered_map<int, int> nextGreaterMap;
        for (int i = 0; i < nums2.size(); i++) {
            if (nextIdx[i] != -1)
                nextGreaterMap[nums2[i]] = nums2[nextIdx[i]];
            else
                nextGreaterMap[nums2[i]] = -1;
        }

        for (int i = 0; i < nums1.size(); i++)
            res[i] = nextGreaterMap[nums1[i]];
        return res;
    }
};
