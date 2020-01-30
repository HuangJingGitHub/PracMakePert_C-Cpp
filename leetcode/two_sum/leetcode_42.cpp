class Solution {
public:
    int trap(vector<int>& height) {
        int left, right, vol = 0, volSum = 0;
        if (height.size() <= 2)
            return 0;
        
        for(left = 0; left < height.size() && height[left] == 0; left++);
        if (left == height.size() || left == height.size() - 1)
            return 0;

        for (; left < height.size() - 2; ){
            if (height[left+1] >= height[left]){
                left++;
                continue;
            }
            else{
                right = left + 2;
                for (vol = height[left] - height[left-1]; right < height.size() &&
                 height[right] < height[left]; right++){
                     vol += height[left] - height[right];
                 }
                 if (right == height.size())
                    break;
                volSum += vol;
                left = right;
            }
        }
        return volSum;
    }  
};
