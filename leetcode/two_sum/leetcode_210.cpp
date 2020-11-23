\\ bfs, interesting
class Solution {
public:
    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
        vector<int> res, indegree(numCourses, 0);
        vector<vector<int>> outAdjacency(numCourses);
        queue<int> feasibleCourse;

        for (auto pre : prerequisites){
            indegree[pre[0]]++;
            outAdjacency[pre[1]].push_back(pre[0]);
        }
        for (int i = 0; i < numCourses; i++)
            if (indegree[i] == 0)
                feasibleCourse.push(i);
        
        while (!feasibleCourse.empty()){
            int feasibleCourseNum = feasibleCourse.size();
            for (int i = 0; i < feasibleCourseNum; i++){
                int curCourse = feasibleCourse.front();
                res.push_back(curCourse);
                feasibleCourse.pop();
                for (int course : outAdjacency[curCourse]){
                    indegree[course]--;
                    if (indegree[course] == 0)
                        feasibleCourse.push(course);
                }
            }
        }
        if (res.size() == numCourses)
            return res;
        else
            return vector<int>();
    }
};
