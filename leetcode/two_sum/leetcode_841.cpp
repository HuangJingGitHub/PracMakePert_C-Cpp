class Solution {
public:
    bool canVisitAllRooms(vector<vector<int>>& rooms) {
        vector<bool> visited(rooms.size(), false);

        dfs(rooms, visited, 0);
        for (bool is_visited : visited)
            if (is_visited == false)
                return false;
        return true;
    }

    void dfs(vector<vector<int>>& rooms, vector<bool>& visited, int i) {
        if (visited[i] == true)
            return;

        visited[i] = true;
        for (int key : rooms[i])
            dfs(rooms, visited, key);
    }
};
