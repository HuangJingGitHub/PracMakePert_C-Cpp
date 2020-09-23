class Solution {
public:
    int findMinDifference(vector<string>& timePoints) {
        vector<int> time(timePoints.size(), 0);

        for (int i = 0; i < timePoints.size(); i++)
            time[i] = stoi(timePoints[i].substr(0, 2)) * 60 + stoi(timePoints[i].substr(3, 2));
        
        sort(time.begin(), time.end());
        int temp = time[0];
        for (int i = 0; i < time.size()-1; i++)  
            time[i] = time[i+1] - time[i];
        time.back() = 24 * 60 - time.back() + temp;

        return *min_element(time.begin(), time.end());
    }
};
