class Solution {
public:
    bool canReach(vector<int>& arr, int start) {
        vector<bool> visited(arr.size(), false);
        queue<int> idx_queue;
        idx_queue.push(start);
        visited[start] = true;

        while (!idx_queue.empty()) {
            int idx = idx_queue.front();
            idx_queue.pop();

            int idx_1 = idx + arr[idx], idx_2 = idx - arr[idx];
            if (idx_1 >= 0 && idx_1 < arr.size()) {
                if (arr[idx_1] == 0)
                    return true;

                if (!visited[idx_1]) {
                    visited[idx_1] = true;
                    idx_queue.push(idx_1);
                }
            }
            if (idx_2 >= 0 && idx_2 < arr.size()) {
                if (arr[idx_2] == 0)
                    return true;

                if (!visited[idx_2]) {
                    visited[idx_2] = true;
                    idx_queue.push(idx_2);
                }
            }            
        }
        return false;
    }
};
