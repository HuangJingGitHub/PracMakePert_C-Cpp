// This is a general structure for a array (containing both negative, positive elements) to find the max product of two elements.
class Solution {
public:
    int maxProduct(vector<int>& nums) {
        if (nums.size() < 2)
            return 0;
        if (nums.size() == 2)
            return (nums[0]-1)*(nums[1]-1);
        
        int posMax1 = 0, posMax2 = 0, negMin1 = 0, negMin2 = 0, temp;
        for (int i = 0; i < nums.size(); i++){
            temp = nums[i] - 1;
            if (temp > 0){
                if (temp <= posMax2)
                    continue;
                else if (temp > posMax1){
                    posMax2 = posMax1;
                    posMax1 = temp;
                }
                else
                    posMax2 = temp;
            }
            else if (temp < 0){
                if (temp >= negMin2)
                    continue;
                else if (temp < negMin1){
                    negMin2 = negMin1;
                    negMin1 = temp;
                }
                else
                    negMin2 = temp;
            }
        }

        return max(posMax1*posMax2, negMin1*negMin2);
    }
};

// For case array only containing positive elements
class Solution {
public:
    int maxProduct(vector<int>& nums) {
        int len = nums.size();
        int max1 = 0, max2 = 0;  // Note max1 >= max2 >= 0
        for (int i = 0; i < len; i++){
            if (nums[i] - 1 > max1){
                max2 = max1;
                max1 = nums[i] - 1;
            }
            else if (nums[i] - 1 > max2)
                max2 = nums[i] - 1;
        }
        return max1 * max2;
    }
};
