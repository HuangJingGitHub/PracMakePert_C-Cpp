// very interesting problem and solution
class Solution {
public:
    bool increasingTriplet(vector<int>& nums) {
        int minNum = INT_MAX;
        int midNum = INT_MAX;

        for (int num : nums) {
            if (num <= minNum)
                minNum = num;
            else if (num <= midNum)
                midNum = num;
            else 
                return true;
        }
        return false;
    }
};
