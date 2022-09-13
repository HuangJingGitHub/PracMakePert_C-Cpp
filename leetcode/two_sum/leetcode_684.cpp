class Solution {
public:
    int FindParent(vector<int>& parent, int index) {
        if (parent[index] == index)
            return index;
        return parent[FindParent(parent, parent[index])];
    }

    void Union(vector<int>& parent, int index1, int index2) {
        parent[FindParent(parent, index1)] = FindParent(parent, index2);
    }

    vector<int> findRedundantConnection(vector<vector<int>>& edges) {
        vector<int> parent(edges.size() + 1, 0);
        for (int i = 1; i <= edges.size(); i++)
            parent[i] = i;
        
        for (auto& edge : edges) {
            int node1 = edge[0], node2 = edge[1];
            if (FindParent(parent, node1) != FindParent(parent, node2))
                Union(parent, node1, node2);
            else
                return edge;
        }

        return {};
    }
};
