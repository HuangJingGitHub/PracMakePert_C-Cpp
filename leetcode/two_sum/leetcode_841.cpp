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

// bfs
class Solution {
public:
    bool canVisitAllRooms(vector<vector<int>>& rooms) {
        vector<bool> visited(rooms.size(), false);

        queue<int> room_keys;
        room_keys.push(0);
        while (room_keys.empty() == false) {
            int cur_room = room_keys.front();
            room_keys.pop();

            visited[cur_room] = true;
            for (int key : rooms[cur_room])
                if (visited[key] == false)
                    room_keys.push(key);
        }
        for (bool is_visited : visited)
            if (is_visited == false)
                return false;
        return true;
    }
};
