class Solution {
public:
    int findLeastNumOfUniqueInts(vector<int>& arr, int k) {
        unordered_map<int, int> numAppearances;
        for (auto num : arr)
            numAppearances[num]++;
        
        vector<int> numTimes(numAppearances.size(), 0);
        int idx = 0;
        for (auto itr = numAppearances.begin(); itr != numAppearances.end(); itr++, idx++)
            numTimes[idx] = itr->second;
        sort(numTimes.begin(), numTimes.end());

        int removed = 0, removedNum = 0;
        for (idx = 0; idx < numTimes.size(); idx++){
            removed += numTimes[idx];
            if (removed <= k)
                removedNum++;
            else
                break;
        }

        return numTimes.size() - removedNum;
    }
};
