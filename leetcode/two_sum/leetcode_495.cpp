class Solution {
public:
    int findPoisonedDuration(vector<int>& timeSeries, int duration) {
        int res = 0, unpositionTime = timeSeries[0] + duration  - 1;

        for (int i = 1; i < timeSeries.size(); i++) {
            if (unpositionTime < timeSeries[i]) 
                res += duration;
            else 
                res += timeSeries[i] - timeSeries[i - 1];
            unpositionTime = timeSeries[i] + duration - 1;
        }
        res += duration;
        
        return res;
    }
};
