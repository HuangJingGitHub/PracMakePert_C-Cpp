class Solution {
public:
    static bool myCompare(const vector<int>& a, const vector<int>& b) {
        return a[0] > b[0] || (a[0] == b[0] && a[1] < b[1]);
    }

    vector<vector<int>> reconstructQueue(vector<vector<int>>& people) {
        vector<vector<int>> res;
        sort(people.begin(), people.end(), myCompare); 

        for (vector<int>& individual : people) {
            //cout << "[" << individual[0] << ", " << individual[1] << "]" << ", ";
            res.insert(res.begin() + individual[1], individual);
        }
        return res;
    }
};
