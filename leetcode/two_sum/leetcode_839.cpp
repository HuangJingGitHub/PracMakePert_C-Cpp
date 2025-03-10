class Solution {
public:
    bool similar(string& str1, string& str2) {
        int diff_cnt = 0;
        for (int i = 0; i < str1.size(); i++) {
            if (str1[i] != str2[i])
                diff_cnt++;
            if (diff_cnt > 2)
                return false;
        }
        return true;
    }

    int numSimilarGroups(vector<string>& strs) {
        int str_num = strs.size();
        vector<vector<int>> adjacency(str_num, vector<int>(str_num, 0));
        for (int i = 1; i < str_num; i++) {
            for (int j = 0; j < i; j++) {
                if (similar(strs[j], strs[i])) {
                    adjacency[j][i] = 1;
                    adjacency[i][j] = 1;
                }
            }
        }

        int group_num = 0;
        vector<bool> visited(str_num, false);
        for (int i = 0; i < str_num; i++) {
            if (!visited[i]) {
                group_num++;
                     
                queue<int> level;
                level.push(i);
                while (!level.empty()) {
                    int str_idx = level.front();
                    level.pop();
                    visited[str_idx] = true;
                    for (int j = i + 1; j < str_num; j++) {
                        if (adjacency[str_idx][j] && !visited[j])
                            level.push(j);
                    }

                }
            }
        }
        return group_num;
    }
};
