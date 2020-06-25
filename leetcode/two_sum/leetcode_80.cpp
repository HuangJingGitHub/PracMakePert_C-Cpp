// interesting, but simple
class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        int len = nums.size();
        if (len <= 2){
            return len;
        }

        int idx = 2;
        for (int i = 2; i < len; i++){
            if (nums[i] != nums[idx-2]){
                nums[idx] = nums[i];
                idx++;
            }
        }

        return idx;
    }
};
