class Solution {
public:
    vector<int> twoSum(vector<int>& numbers, int target) {
        int left = 0, right = numbers.size() - 1, sum;
        while (left < right) {
            sum = numbers[left] + numbers[right];
            if (sum > target)
                right--;
            else if (sum < target)
                left++;
            else
                break;
        }
        return vector<int>{left + 1, right + 1};
    }
};
