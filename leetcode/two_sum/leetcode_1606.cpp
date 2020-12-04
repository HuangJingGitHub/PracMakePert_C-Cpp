class Solution {
public:
    vector<int> busiestServers(int k, vector<int>& arrival, vector<int>& load) {
        unordered_map<int, int> requestLog;
        vector<int> serverFreeTime(k, 0), res;
        int maxRequest = 0, minNextTime = INT_MAX;
    
        for (int i = 0; i < arrival.size(); i++) {
            if (i >= k && minNextTime > arrival[i] + load[i])
                continue;
            for (int idx = i % k, j = 0; j < k; j++, idx = (i + j) % k) {
                if (serverFreeTime[idx] <= arrival[i]) {
                    serverFreeTime[idx] = arrival[i] + load[i];
                    requestLog[idx]++;
                    maxRequest = (maxRequest > requestLog[idx]) ? maxRequest : requestLog[idx];
                    minNextTime = min(minNextTime, arrival[i] + load[i]);
                    break;
                }
            }
        }

        for (auto itr = requestLog.begin(); itr != requestLog.end(); itr++) {
            if (itr->second == maxRequest)
                res.push_back(itr->first);
        }
        return res;
    }
};
