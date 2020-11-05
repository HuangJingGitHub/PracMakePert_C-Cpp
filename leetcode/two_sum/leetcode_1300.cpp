class Solution {
public:
    int findBestValue(vector<int>& arr, int target) {
        if (arr.empty())
            return 0;
        int res, avgTarget = target / arr.size();
        if ((avgTarget + 1) * arr.size() - target < target - avgTarget * arr.size())  // best average can be avgTarget or avgTarget + 1. Or use round()
            avgTarget++;

        sort(arr.begin(), arr.end());
        if (arr.front() >= avgTarget)
            return avgTarget;
        else if (arr.back() <= avgTarget)
            return arr.back();
        
        int diff = 0, segPos = 0;
        for (int i = 0; i < arr.size(); i++){
            if (arr[i] <= avgTarget)
                diff += (arr[i] - avgTarget);
            else{
                segPos = i;
                break;
            }
        }

        cout << avgTarget << " -\n";
        res = avgTarget;
        for (int i = segPos; i < arr.size(); i++){
            int addNum = (arr[i] - avgTarget) * (arr.size() - i);
            if (abs(diff + addNum) <= abs(diff)){
                diff += addNum;
                res = arr[i];
            }
            else
                break;
        }
        return res;
    }
};
