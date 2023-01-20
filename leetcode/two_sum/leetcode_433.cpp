class Solution {
public:
    bool canMutate(string& a, string& b) {
        int differentNum = 0;
        for (int i = 0; i < 8; i++)
            if (a[i] != b[i])
                differentNum += 1;
        
        return differentNum == 1;
    }


    int minMutation(string startGene, string endGene, vector<string>& bank) {
        bank.push_back(startGene);
        int start = bank.size() - 1, goal;
        vector<vector<int>> edge(bank.size());

        for (int i = 0; i < bank.size(); i++) {
            if (bank[i] == endGene)
                goal = i;
            for (int j = i + 1; j < bank.size(); j++) {
                if (canMutate(bank[i], bank[j])) {
                    edge[i].push_back(j);
                    edge[j].push_back(i);
                }
            }
        }

        // BFS
        queue<pair<int, int>> que;
        vector<bool> visited(bank.size(), false);
        que.push(make_pair(start, 0));
        visited[start] = true;
        while (que.empty() == false) {
            int index = que.front().first, cost = que.front().second;
            que.pop();
            for (auto it = edge[index].begin(); it != edge[index].end(); it++) {
                if (*it == goal)
                    return cost + 1;
                
                if (visited[*it] == false) {
                    que.push(make_pair(*it, cost + 1));
                    visited[*it] = true;
                }
            }
        }

        return -1;
    }
};
