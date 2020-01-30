// There is good article for the discussion of the solution. Refer to that.
class Solution {
public:
    int trap(vector<int>& height) {
        if ( height.size() < 3)
            return 0;

        int volSum = 0;
        vector<int> max_left(height.size()), max_right(height.size());

        for (int i = 1; i < height.size(); i++){
            max_left[i] = max(height[i-1], max_left[i-1]);
        }
        for(int i = height.size() - 2; i >= 0; i--){
            max_right[i] = max(height[i+1], max_right[i+1]);
        }

        for (int i = 1; i < height.size() - 1; i++){
            int minHeight = min(max_left[i], max_right[i]);
            if (minHeight > height[i])
                volSum += minHeight - height[i];
        }
        return volSum;
    }  
};
