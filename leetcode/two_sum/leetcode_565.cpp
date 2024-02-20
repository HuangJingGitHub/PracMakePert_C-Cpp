class Solution {
public:
    int arrayNesting(vector<int>& nums) {
        int res = 0;
        for (int i = 0; i < nums.size(); i++) {
            if (nums[i] == -1)
                continue;
            
            int next = nums[i], cur_size = 1;
            while (next != i) {
                int temp = nums[next];
                nums[next] = -1;
                cur_size++;
                next = temp;
            }
            res = max(res, cur_size);
        }
        return res;
    }
};
