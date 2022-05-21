class Solution {
public:
    void sortColors(vector<int>& nums) {
        // There are some details of the index change to deal with cases like [2,2,1],[1,2,0].
        // Change in-place
        for(int i = 0, counter = 0; counter < nums.size(); counter++){
            if (nums[i] == 1){
                i++;
            }
            else if (nums[i] == 0){
                move(nums.begin(), nums.begin()+i, nums.begin()+1);
                nums[0] = 0;
                i++;
            }
            else{
                if (i == nums.size() - 1)
                    return;

                move(nums.begin()+i+1, nums.end(), nums.begin()+i);
                nums.back() = 2;
            }
        }
    }
};


class Solution {
public:
    void sortColors(vector<int>& nums) {
        int zeroEnd = -1, twoStart = nums.size();

        int i = 0;
        while (i < twoStart) {
            if (nums[i] == 0) {
                swap(nums[zeroEnd + 1], nums[i]);
                zeroEnd++;
                i++;
            }
            else if (nums[i] == 1)
                i++;
            else if (nums[i] == 2) {
                swap(nums[i], nums[twoStart - 1]);
                twoStart--;
            }         

        }
    }
};
