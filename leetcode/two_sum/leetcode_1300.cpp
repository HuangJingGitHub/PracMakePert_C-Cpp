// Sorting can help much.
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
                res = target / (arr.size() - i);
                if (target - res * (arr.size() - i) > (res + 1) * (arr.size() - i) - target)
                    res++;
                return res;
            }
        }
        return res;
    }
};
};
