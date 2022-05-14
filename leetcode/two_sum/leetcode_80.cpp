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


class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        if (nums.size() <= 2)
            return nums.size();
            
        int newIdx = 1, idx = 2;
        for (;idx < nums.size(); idx++) {
            if (nums[newIdx] == nums[idx] && newIdx > 0 && nums[newIdx - 1] == nums[newIdx])
                    continue;
            else {
                newIdx++;
                nums[newIdx] = nums[idx];
            }
        }
        return newIdx + 1;
    }
};
