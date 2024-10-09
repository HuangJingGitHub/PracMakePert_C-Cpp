class Solution {
public:
    static bool XYCompare(vector<int>& a, vector<int>& b) {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    }

    bool TurnRight(vector<int>& p_1, vector<int>& p_2, vector<int>& p_3) {
        vector<int> v_12{p_2[0] - p_1[0], p_2[1] - p_1[1]},
                    v_23{p_3[0] - p_2[0], p_3[1] - p_2[1]};
        int cross_product = v_12[0] * v_23[1] - v_23[0] * v_12[1];
        return cross_product <= 0;
    }

    vector<vector<int>> outerTrees(vector<vector<int>>& trees) {
        int tree_num = trees.size();
        if (tree_num <= 3)
            return trees;

        std::sort(trees.begin(), trees.end(), XYCompare);
        stack<vector<int>> upper;
        upper.push(trees[0]);
        upper.push(trees[1]);

        vector<int> p_right, p_mid, p_left;
        for (int i = 2; i < tree_num; i++) {
            upper.push(trees[i]);

            p_right = upper.top();
            upper.pop();
            p_mid = upper.top();
            upper.pop();
            p_left = upper.top();
            upper.push(p_mid);
            upper.push(p_right);

            while (TurnRight(p_left, p_mid, p_right) == false) {
                upper.pop();
                upper.pop();
                upper.push(p_right);

                if (upper.size() > 2) {
                    p_right = upper.top();
                    upper.pop();
                    p_mid = upper.top();
                    upper.pop();
                    p_left = upper.top();
                    upper.push(p_mid);
                    upper.push(p_right);
                }
                else
                    break;
            }
        }

        stack<vector<int>> lower;
        lower.push(trees.back());
        lower.push(trees[tree_num - 2]);

        for (int i = tree_num - 3; i >= 0; i--) {
            lower.push(trees[i]);

            p_left = lower.top();
            lower.pop();
            p_mid = lower.top();
            lower.pop();
            p_right = lower.top();
            lower.push(p_mid);
            lower.push(p_left);

            while (TurnRight(p_right, p_mid, p_left) == false) {
                lower.pop();
                lower.pop();
                lower.push(p_left);

                if (lower.size() > 2) {
                    p_left = lower.top();
                    lower.pop();
                    p_mid = lower.top();
                    lower.pop();
                    p_right = lower.top();
                    lower.push(p_mid);
                    lower.push(p_left);
                }
                else
                    break;
            }
        }

        vector<vector<int>> upper_vec(upper.size());
        set<vector<int>> vertex_set;
        for (int i = 0; i < upper_vec.size(); i++) {
            upper_vec[i] = upper.top();
            vertex_set.insert(upper.top());
            upper.pop();
        }
        std::reverse(upper_vec.begin(), upper_vec.end());
        
        vector<vector<int>> lower_vec(lower.size() - 2);
        lower.pop();
        for (int i = 0; i < lower_vec.size(); i++) {
            lower_vec[i] = lower.top();
            lower.pop();
        }
        std::reverse(lower_vec.begin(), lower_vec.end());
        for (vector<int>& p : lower_vec) {
            if (vertex_set.count(p) == 0)   // Handle cases where all vertices are on a line. upper and lower
                upper_vec.push_back(p);     // have identical vertices.
        }
        return upper_vec;
    }
};
