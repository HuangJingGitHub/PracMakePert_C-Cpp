class UnionFind {
public:
    vector<int> parent;

    UnionFind(int n) {
        parent.resize(n);
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    void unionSet(int idx_1, int idx_2) {
        parent[find(idx_2)] = find(idx_1);
    }

    int find(int idx) {
        if (parent[idx] != idx)
            parent[idx] = find(parent[idx]);
        return parent[idx];
    }
};

class Solution {
public:
    bool equationsPossible(vector<string>& equations) {
        UnionFind groups(26);
        for (auto& eqn : equations) {
            if (eqn[1] == '=') {
                int x = eqn[0] - 'a', y = eqn[3] - 'a',
                    min_v = min(x, y), max_v = max(x, y);
                groups.unionSet(min_v, max_v);
            }
        }
        
        for (auto& eqn: equations) {
            if (eqn[1] == '!') {
                int x = eqn[0] - 'a', y = eqn[3] - 'a';
                if (groups.find(x) == groups.find(y))
                    return false;
            }
        }
        return true;
    }
};
