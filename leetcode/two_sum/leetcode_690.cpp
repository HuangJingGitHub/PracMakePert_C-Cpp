/*
// Definition for Employee.
class Employee {
public:
    int id;
    int importance;
    vector<int> subordinates;
};
*/

class Solution {
public:
    int getImportance(vector<Employee*> employees, int id) {
        int res = 0;
        map<int, vector<int>> subordinates_set;
        map<int, int> importance_set;

        for (auto& e : employees) {
            int id = e->id;
            importance_set[id] = e->importance;
            subordinates_set[id] = e->subordinates;
        }

        queue<int> id_que;
        id_que.push(id);
        while(!id_que.empty()) {
            int cur_id = id_que.front();
            id_que.pop();
            res += importance_set[cur_id];
            for (int sub : subordinates_set[cur_id])
                id_que.push(sub);
        }

        return res;
    }
};
