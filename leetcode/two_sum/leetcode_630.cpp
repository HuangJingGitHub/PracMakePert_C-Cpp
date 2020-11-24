// can refer to the offical solution - priority queue
class Solution {
public:
    static bool cmpDay(vector<int>& a, vector<int>&b){
        assert(a.size() == 2);
        assert(b.size() == 2);
        return a[1] < b[1];
    }

    struct timeComparator{
        bool operator () (vector<int>& a, vector<int>&b){
            assert(a.size() == 2);
            assert(b.size() == 2);
            return a[0] < b[0];
        }
    };

    int scheduleCourse(vector<vector<int>>& courses) {
        sort(courses.begin(), courses.end(), cmpDay);
        priority_queue<vector<int>, vector<std::vector<int>>, timeComparator> courseQueue;
        //priority_queue<vector<int>> courseQueue; // Feasible, as by default, the comparator will compare the first element in vector.

        courseQueue.push(courses[0]);
        int curDay = courses[0][0];
        for (int i = 1; i < courses.size(); i++){
            if (curDay + courses[i][0] <= courses[i][1]){
                courseQueue.push(courses[i]);
                curDay += courses[i][0];
            }
            else if (courseQueue.top()[0] > courses[i][0]){
                curDay -= courseQueue.top()[0];
                curDay += courses[i][0];
                courseQueue.pop();
                courseQueue.push(courses[i]);
            }
        }
        return courseQueue.size();
    }
};
