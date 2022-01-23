// bucket sort, linear time to find max gap
class Solution {
private:
struct bucket{
    int min = INT_MAX;
    int max = INT_MIN;
};
// Or use pair<int, int>
    
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


// implementation for the same algorithm
class Solution {
public:
    int maximumGap(vector<int>& nums) {
        if (nums.size() < 2)
            return 0;

        int res = 0;
        int maxNum = *std::max_element(nums.begin(), nums.end()),
            minNum = *std::min_element(nums.begin(), nums.end());
        
        int bucketLen = max(1, (maxNum - minNum) / (int)nums.size()),
            bucketNum = (maxNum - minNum) / bucketLen + 1;
        
        vector<pair<int, int>> bucketVec(bucketNum, pair<int, int>(INT_MAX, INT_MIN));
        for (int& num : nums) {
            int idx = (num - minNum) / bucketLen;
            bucketVec[idx].first = min(num, bucketVec[idx].first);
            bucketVec[idx].second = max(num, bucketVec[idx].second);
        }

        int preMax = bucketVec[0].second;
        for (int i = 0; i < bucketNum; i++) {
            if (bucketVec[i].first == INT_MAX)
                continue;
            
            res = max(res, bucketVec[i].first - preMax); 
            preMax = bucketVec[i].second;
        }
        return res;
    }
};
