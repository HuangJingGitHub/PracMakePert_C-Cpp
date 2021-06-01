class Solution {
public:
    vector<int> singleNumber(vector<int>& nums) {
        long int xorRes = 0;
        for (int num : nums)
            xorRes ^= num;
        long int mask = xorRes & (-xorRes);

        vector<int> res(2, 0);
        for (int num : nums) {
            if ((num & mask) == 0)
                res[0] ^= num;
            else
                res[1] ^= num;
        }

        return res;
    }
};
