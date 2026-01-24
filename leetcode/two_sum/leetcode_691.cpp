class Solution {
public:
    int minStickers(vector<string>& stickers, string target) {
        int n = stickers.size();

        // 1. 把每张 sticker 转成 26 维字母频次
        vector<vector<int>> sc(n, vector<int>(26, 0));
        for (int i = 0; i < n; ++i) {
            for (char c : stickers[i]) {
                sc[i][c - 'a']++;
            }
        }

        unordered_map<string, int> memo;

        function<int(const string&)> dfs = [&](const string& remain) -> int {
            if (remain.empty()) return 0;
            if (memo.count(remain)) return memo[remain];

            vector<int> rc(26, 0);
            for (char c : remain) rc[c - 'a']++;

            int res = INT_MAX;

            // 2. 尝试每一张贴纸
            for (int i = 0; i < n; ++i) {
                // 剪枝：如果这张贴纸对 remain 的第一个字符没贡献，跳过
                char first_char = remain[0];
                if (sc[i][first_char - 'a'] == 0) continue;

                // 3. 计算 new_remain = remain - sticker[i]
                string new_remain;
                for (int c = 0; c < 26; ++c) {
                    int cnt = rc[c] - sc[i][c];
                    for (int k = 0; k < max(0, cnt); ++k) {
                        new_remain.push_back('a' + c);
                    }
                }

                int sub = dfs(new_remain);
                if (sub != -1) {
                    res = min(res, 1 + sub);
                }
            }

            memo[remain] = (res == INT_MAX ? -1 : res);
            return memo[remain];
        };

        return dfs(target);        
    }
};
