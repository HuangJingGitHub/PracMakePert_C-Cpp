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
