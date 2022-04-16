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
int removeDuplicates(vector<int>& nums) {
	if (nums.size() < 2) return nums.size();
	int j = 0;
	for (int i = 1; i < nums.size(); i++)
		if (nums[j] != nums[i]) {
            j++;
            nums[j] = nums[i];
        }
	return ++j;
    }
};
