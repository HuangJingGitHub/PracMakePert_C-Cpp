class Solution {
public:
    vector<int> lexicalOrder(int n) {
        vector<int> res;
        
        for (int i = 1; i <= 9; i++)
            dfs(i, n, res);
        return res;
    }

    void dfs(int num, int n, vector<int>& res) {
        if (num > n)
            return;
        
        res.push_back(num);
        num *= 10;
        if (num > n)
            return;
      
        for (int i = 0; i <= 9; i++)
            dfs(num + i, n, res);
    }
};
