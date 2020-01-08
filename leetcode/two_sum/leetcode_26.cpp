class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        if (nums.size() <= 1)
            return nums.size();

        for (auto pos = nums.begin()+1; pos != nums.end(); ){
            if (*pos == *(pos-1))
                nums.erase(pos);
            else
                pos++;
        }
        return nums.size();
    }
};

// Faster with fewer manipulations.
class Solution {
public:
    int removeElement(vector<int>& nums, int val) {
        int newLength = 0;
        for (int i = 0; i < nums.size(); i++)
         if (nums[i] != val) nums[newLength++] = nums[i];  
        return newLength;
    }
};
