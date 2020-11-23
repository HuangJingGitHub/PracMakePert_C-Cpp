// bfs
class Solution {
public:
    bool canFinish(int numCourses, vector<vector<int>>& prerequisites) {
        int courseTaken = 0;
        vector<int> indegree(numCourses, 0);
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
                feasibleCourse.pop();
                courseTaken++;
                for (int course : outAdjacency[curCourse]){
                    indegree[course]--;
                    if (indegree[course] == 0)
                        feasibleCourse.push(course);
                }
            }
        }
        return courseTaken == numCourses;
    }
};
