// The bitwise operation is interesting.
class Solution {
public:
    int singleNumber(vector<int>& nums) {
        vector<int> resBin(32, 0);  // binary representation of the result
        int res = 0;
        
        for (int num : nums)
            for (int i = 31; i >= 0 ; i--) {
                resBin[i] += num & 1;  // x & 1 to get the last bit of x
                num >>= 1;
            }
        
        for (int i = 0; i < 32; i++) {
            res <<= 1;
            res |= resBin[i] % 3;  // The mod result 0 or 1 is equal to the bit balue of the unique number in array. x |= 0 --> last bit intact, x |= 1 --> last bit 1
        }
        return res;
    }
};
