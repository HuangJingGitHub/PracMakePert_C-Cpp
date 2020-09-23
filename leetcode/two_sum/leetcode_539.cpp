class Solution {
public:
    int findMinDifference(vector<string>& timePoints) {
        vector<int> time(timePoints.size(), 0);

        for (int i = 0; i < timePoints.size(); i++)
            time[i] = stoi(timePoints[i].substr(0, 2)) * 60 + stoi(timePoints[i].substr(3, 2));
        
        sort(time.begin(), time.end());
        int temp = time.back();
        for (int i = time.size()-1; i > 0; i--)  // A trap to get the difference is loop from beginning, which will lose original time
            time[i] = time[i] - time[i-1];
        time[0] = time[0] + 24 * 60 - temp;

        return *min_element(time.begin(), time.end());
    }
};
