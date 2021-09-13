class Solution {
public:
    map<string, priority_queue<string, vector<string>, std::greater<string>>> log;
    
    vector<string> findItinerary(vector<vector<string>>& tickets) {
        vector<string> res;
        for (auto& line : tickets)
            log[line[0]].push(line[1]);
        
        dfs("JFK", res);
        reverse(res.begin(), res.end());
        return res;
    }

    void dfs(string curStr, vector<string>& res) {
        while (log[curStr].size() > 0) {
            string temp = log[curStr].top();
            log[curStr].pop();
            dfs(temp, res);
        }
        res.push_back(curStr);
    }
};
