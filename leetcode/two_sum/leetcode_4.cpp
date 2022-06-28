class Solution {
public:
    double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
        nums1.insert(nums1.end(), nums2.begin(), nums2.end());
        sort(nums1.begin(), nums1.end());
        //nums1.erase(unique(nums1.begin(), nums1.end()), nums1.end());
        //for (int i = 0; i < nums1.size(); i++) cout << nums1[i] << " ";
        if (nums1.size() % 2 == 0)
            return (nums1[nums1.size() / 2] + nums1[nums1.size() / 2 - 1]) / 2.0;
        else
            return nums1[(nums1.size() - 1) / 2];
    }
};
