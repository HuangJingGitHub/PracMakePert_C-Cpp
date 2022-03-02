class Solution {
public:
    vector<double> calcEquation(vector<vector<string>>& equations, vector<double>& values, vector<vector<string>>& queries) {
        unordered_map<string, vector<string>> adjacencyMap;
        unordered_map<string, double> strPairToValue;

        for (int i = 0; i < equations.size(); i++) {
            string dividend = equations[i][0],
                    divisor = equations[i][1];
            adjacencyMap[dividend].push_back(divisor);
            adjacencyMap[divisor].push_back(dividend);
            strPairToValue[dividend + divisor] = values[i];
            strPairToValue[divisor + dividend] = 1 / values[i];
        }
        cout << "OK\n";
        vector<double> res(queries.size(), 1);
        for (int i = 0; i < queries.size(); i++) {
            if (queries[i][0] == queries[i][1]) {
                continue;
            }
            vector<string> path;
            dfs(adjacencyMap, path, queries[i][0], queries[i][1]);
            if (path.empty() == true)
                res[i] = -1.0;
            for (int i = 0; i < path.size() - 1; i++)
                res[i] *= strPairToValue[path[i] + path[i + 1]];
        }
        return res;
    }

    void dfs(unordered_map<string, vector<string>>& adjacencyMap, vector<string>& path, string start, string end) {
        if (adjacencyMap.find(start) == adjacencyMap.end())
            return;

        path.push_back(start);
        for (int i = 0; i < adjacencyMap[start].size(); i++) {
            path.push_back(adjacencyMap[start][i]);
            if (adjacencyMap[start][i] == end)
                return;
            else {
                dfs(adjacencyMap, path, adjacencyMap[start][i], end);
                path.pop_back();
            }
        }
    }
};
