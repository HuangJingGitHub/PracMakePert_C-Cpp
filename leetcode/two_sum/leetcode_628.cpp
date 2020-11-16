// using sorted array, O(nlog(n)) time.
class Solution {
public:
    int maximumProduct(vector<int>& nums) {
        sort(nums.begin(), nums.end());

        int len = nums.size();
        int temp1 = nums[0] * nums[1] * nums.back(), 
            temp2 = nums[len - 1] * nums[len - 2] * nums[len - 3];
        if (temp1 >= temp2)
            return temp1;
        else
            return temp2;
    }
};

// linear scan for elements of interest, O(n) time.
class Solution {
public:
    int maximumProduct(vector<int>& nums) {
        priority_queue<int> min2;  // max heap to store min 2 numbers
        priority_queue<int, vector<int>, greater<int>> max3;  // min heap to store max 3 numbers

        for (int i = 0; i < 2; i++)
            min2.push(nums[i]);
        for (int i = 0; i < 3; i++)
            max3.push(nums[i]);
        
        for (int i = 2; i < nums.size(); i++){
            if (nums[i] < min2.top()){
                min2.pop();
                min2.push(nums[i]);
            }
            if (i > 2 && nums[i] > max3.top()){
                max3.pop();
                max3.push(nums[i]);
            }
        }

        int temp1 = 1, temp2 = 1;
        while (!min2.empty()){
            temp1 *= min2.top();
            min2.pop();
        }

        while (max3.size() != 1){
            temp2 *= max3.top();
            max3.pop();
        }
        temp1 *= max3.top();
        temp2 *= max3.top();
        
        if (temp1 >= temp2)
            return temp1;
        else
            return temp2;
    }
};
