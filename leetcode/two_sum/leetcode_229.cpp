// Moore voting algorithm
class Solution {
public:
    vector<int> majorityElement(vector<int>& nums) {
        if (nums.size() <= 1)
            return nums;
        vector<int> res;

        int cand1 = nums[0], cand2 = nums[1], 
            cnt1 = 0, cnt2 = 0;
        
        for (int num : nums) {
            if (num == cand1) {
                cnt1++;
                continue;
            }
            else if (num == cand2) {
                cnt2++;
                continue;
            }
            if (cnt1 == 0) {
                cand1 = num;
                cnt1++;
                continue;
            }
            if (cnt2 == 0) {
                cand2 = num;
                cnt2++;
                continue;
            }
            cnt1--;
            cnt2--;
        }

        cnt1 = 0;
        cnt2 = 0;
        for (int num : nums) {
            if (num == cand1)
                cnt1++;
            else if (num == cand2)
                cnt2++;
        }
        if (cnt1 > nums.size() / 3)
            res.push_back(cand1);
        if (cnt2 > nums.size() / 3)
            res.push_back(cand2);
        return res;
    }
};
