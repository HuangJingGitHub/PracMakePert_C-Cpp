// brute force, O(n^2). Time limit is vilated.
class Solution {
public:
    int maxArea(vector<int>& height) {
        int maxArea = 0, area =  0, n = height.size();

        for (int i = 0; i < n-1; i++)
            for (int j = i + 1; j < n; j++) {
                area  = min(height[i], height[j]) * (j - i);
                maxArea = maxArea > area ? maxArea : area;
            }
        return maxArea; 
    }
};

// two pointer linear traversal
class Solution {
public:
    int maxArea(vector<int>& height) {
        int res = 0, left = 0, right = height.size() - 1;
        while (left < right) {
            res = max(res, min(height[left], height[right]) * (right - left));
            if (height[left] <= height[right])
                left++;
            else
                right--;
        }
        return res;
    }
};
