class Solution {
public:
    static bool cmp(pair<int, int>& a, pair<int, int>& b) {
        return a.second > b.second;
    }

    vector<int> topKFrequent(vector<int>& nums, int k) {
        vector<int> res;
        unordered_map<int, int> frequencyLog;
        for (int& num : nums)
            frequencyLog[num]++;
        
        priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(&cmp)> minHeap(cmp); 
        auto it = frequencyLog.begin();
        
        for (int i = 0; i < k; it++, i++)
            minHeap.push(*it);

        while (it != frequencyLog.end()) {
            if ((*it).second > minHeap.top().second) {
                minHeap.pop();
                minHeap.push(*it);
            }
            it++;
        }

        while (!minHeap.empty()) {
            res.push_back(minHeap.top().first);
            minHeap.pop();
        }
        return res;
    }
};
