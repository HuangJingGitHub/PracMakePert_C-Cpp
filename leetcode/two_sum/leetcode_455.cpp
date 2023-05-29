class Solution {
public:
    int findContentChildren(vector<int>& g, vector<int>& s) {
        int res = 0;
        sort(g.begin(), g.end());
        sort(s.begin(), s.end());

        for (int s_idx = 0, g_idx = 0; s_idx < s.size() && g_idx < g.size(); ) {
            if (s[s_idx] < g[g_idx]) {
                s_idx++;
            }
            else {
                res++;
                s_idx++;
                g_idx++;
            }
        }
        return res;
    }
};
