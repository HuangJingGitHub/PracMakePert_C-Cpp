// traceback 
class Solution {
public:
    vector<vector<int>> res;
    
    void traceback(int n, int k, int begin, vector<int> pre)
    {
        if (pre.size() == k){
            res.push_back(pre);
            return;
        }

        for (int i = begin; i <= n; i++){
            pre.push_back(i);
            traceback(n, k, i+1, pre);
            pre.pop_back();
        }
    }

    vector<vector<int>> combine(int n, int k) {
        if (n <= 0 || k <= 0 || n < k){
            return res;
        }

        vector<int> path;
        traceback(n, k, 1, path);
        return res;
    }
};
