class Solution {
public:
    int leastInterval(vector<char>& tasks, int n) {
        map<char, int> task_num_map;
        for (char& task : tasks) {
            task_num_map[task]++;
        }

        int task_types = task_num_map.size(), 
            max_task_num = -1, equal_max = 1,
            total_task_num = 0;
        for (auto it = task_num_map.begin(); it != task_num_map.end(); it++) {
            total_task_num += it->second;
            if (max_task_num == it->second) {
                equal_max++;
            }
            else if (max_task_num < it->second) {
                max_task_num = it->second;
                equal_max = 1;
            }
        }

        return max((max_task_num - 1) * (n + 1) + equal_max, total_task_num);
    }
};
