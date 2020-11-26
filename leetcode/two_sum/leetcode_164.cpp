// bucket sort, linear time to find max gap
class Solution {
private:
struct bucket{
    int min = INT_MAX;
    int max = INT_MIN;
};

public:
    int maximumGap(vector<int>& nums) {
        if (nums.size() < 2)
            return 0;
        
        int minNum = nums[0], maxNum = nums[0], res = 0;
        for (int num : nums){
            minNum = min(minNum, num);
            maxNum = max(maxNum, num);
        }
        int intSize = nums.size();
        int bucketSize = max(1, (maxNum - minNum) / (intSize - 1));
        vector<bucket> bucketVec((maxNum - minNum) / bucketSize + 1);
        
        for (int num : nums){
            int idx = (num - minNum) / bucketSize;
            bucketVec[idx].min = min(bucketVec[idx].min, num);
            bucketVec[idx].max = max(bucketVec[idx].max, num);
        }
        
        int preMax = bucketVec[0].max;
        for (int i = 1; i < bucketVec.size(); i++){
            if (bucketVec[i].min == INT_MAX)
                continue;
            res = max(res, bucketVec[i].min - preMax);
            preMax = bucketVec[i].max;
        }
        return res;
    }
};
