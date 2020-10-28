class Solution {
public:
    vector<vector<int>> criticalConnections(int n, vector<vector<int>>& connections) {
        vector<vector<int>> res;
        set<int> criticalNode;
        unordered_map<int, set<int>> adjacency;
        for (int i = 0; i < connections.size(); i++){
            adjacency[connections[i][0]].insert(connections[i][1]);
            adjacency[connections[i][1]].insert(connections[i][0]);
        }

        int time = 1;
        stack<int> nodeStack;
        vector<int> idx(n, 0), depth(n, 0); // Nodes in a circle are at the same depth.
        for (int i = 0; i < n; i++){
            if (idx[i] == 0)
                DFS(adjacency, nodeStack, criticalNode, idx, depth, time, i);
        }

        for (vector<int> x : connections){
            if (criticalNode.find(x[0]) != criticalNode.end() || criticalNode.find(x[1]) != criticalNode.end())
                res.push_back(x);
        }
        return res;
    }

    void DFS(unordered_map<int, set<int>>& adjacency, stack<int>& nodeStack, set<int>& criticalNode, vector<int>& idx, vector<int>& depth, int& time, int i){
        nodeStack.push(i);
        idx[i] = time;
        depth[i] = time++;
        for (int id = 0; id < idx.size(); id++)
            cout << "(" << idx[id] << ", " << depth[id] << ") ";
        cout << "\n";

        for (set<int>::iterator it = adjacency[i].begin(); it != adjacency[i].end(); it++){
            if (idx[*it] == 0)
                DFS(adjacency, nodeStack, criticalNode, idx, depth, time, *it);
            depth[i] = min(depth[i], depth[*it]);
        }
        if (idx[i] == depth[i]){
            cout << "i, stack.top() = " << i << ' ' << nodeStack.top() << endl;
            nodeStack.pop();
            if (nodeStack.empty() || depth[nodeStack.top()] != depth[i])
                criticalNode.insert(i);
            else
                while (!nodeStack.empty() && depth[nodeStack.top()] == depth[i])
                    nodeStack.pop();
        }
    }
};
