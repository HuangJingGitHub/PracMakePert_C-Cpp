class Solution {
public:
    int fourSumCount(vector<int>& nums1, vector<int>& nums2, vector<int>& nums3, vector<int>& nums4) {
        int res = 0;
        map<int, int> sum_map_1_2;
        for (int& num_1 : nums1)
            for (int& num_2 : nums2)
                sum_map_1_2[num_1 + num_2]++;
        
        for (int& num_3 : nums3)
            for (int& num_4 : nums4)
                res += sum_map_1_2[-(num_3 + num_4)];
        return res;
    }
};
