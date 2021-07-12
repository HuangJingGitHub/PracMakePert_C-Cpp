class Solution {
public:
    int findDuplicate(vector<int>& nums) {
        int n = nums.size() - 1;
        unordered_map<int, int> appearanceLog;

        for (int& num : nums) {
            if (appearanceLog.find(num) == appearanceLog.end())
                appearanceLog.insert({num, 1});
            else
                return num;
        }
        return 1;
    }
};


// Treat the array as a linked list and use fast-slow pointer.
class Solution {
public:
    int findDuplicate(vector<int>& nums) {
        int res = 0;;
        for (int fast = 0; res != fast || fast == 0; ) {
            res = nums[res];
            fast = nums[nums[fast]];
        }

        for (int i = 0; res != i; i = nums[i]) {
            res = nums[res];
        }
        return res;
    }
};
