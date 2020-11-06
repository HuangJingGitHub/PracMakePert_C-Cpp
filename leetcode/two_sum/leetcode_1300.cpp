  
// sorting can help much.
class Solution {
public:
    int findBestValue(vector<int>& arr, int target) {
        int res = 0;
        
        sort(arr.begin(), arr.end());
        for (int i = 0; i < arr.size(); i++){
            int maxSum = arr[i] * (arr.size() - i);
            if (maxSum < target){
                target -= arr[i];
                res = arr[i];
            }
            else if (maxSum == target)
                return arr[i];
            else{
                int targetAvg = target / (arr.size() - i);
                if (target - targetAvg * (arr.size() - i) > (targetAvg + 1) * (arr.size() - i) - target)
                    targetAvg++;
                return targetAvg;
            }
        }
        return res;
    }
};
