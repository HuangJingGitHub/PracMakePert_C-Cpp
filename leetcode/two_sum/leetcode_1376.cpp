class Solution {
public:
    int numOfMinutes(int n, int headID, vector<int>& manager, vector<int>& informTime) {
        int res = 0;
        vector<int> time(n, 0);
        vector<vector<int>> subordinates(n);
        for (int i = 0; i < n; i++)
            if (i != headID)
                subordinates[manager[i]].push_back(i);
        
        queue<int> level;
        level.push(headID);
        while (level.empty() == false) {
            int id = level.front();
            level.pop();
            for (int sub : subordinates[id]) {
                time[sub] = time[id] + informTime[id];
                res = max(res, time[sub]);
                level.push(sub);
            }
        }
        return res;
    }
};
