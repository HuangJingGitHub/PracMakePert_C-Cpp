class Solution {
public:
    static bool XYCompare(vector<int>& a, vector<int>& b) {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    }

    bool TurnRight(vector<int>& p_1, vector<int>& p_2, vector<int>& p_3) {
        vector<int> v_12{p_2[0] - p_1[0], p_2[1] - p_1[1]},
                    v_23{p_3[0] - p_2[0], p_3[1] - p_2[1]};
        int cross_product = v_12[0] * v_23[1] - v_23[0] * v_12[1];
        return cross_product < 0;
    }

    vector<vector<int>> outerTrees(vector<vector<int>>& trees) {
        int tree_num = trees.size();
        if (tree_num <= 3)
            return trees;

        std::sort(trees.begin(), trees.end(), XYCompare);
        stack<vector<int>> upper;
        upper.push(trees[0]);
        upper.push(trees[1]);

        std::cout << "OK1\n"; 
        vector<int> p_right, p_mid, p_left;
        for (int i = 2; i < tree_num; i++) {
            cout << i << "\n";
            upper.push(trees[i]);

            p_right = upper.top();
            upper.pop();
            p_mid = upper.top();
            upper.pop();
            p_left = upper.top();
            upper.push(p_mid);
            upper.push(p_right);

            while (upper.size() > 2 &&
                   TurnRight(p_left, p_mid, p_right) == false) {
                upper.pop();
                upper.pop();
                upper.push(p_right);

                p_right = upper.top();
                upper.pop();
                p_mid = upper.top();
                upper.pop();
                p_left = upper.top();
                upper.push(p_mid);
                upper.push(p_right);
            }
        }
        std::cout << "OK\n"; 

        stack<vector<int>> lower;
        upper.push(trees.back());
        upper.push(trees[tree_num - 2]);

        for (int i = tree_num - 3; i >= 0; i--) {
            lower.push(trees[i]);

            p_left = lower.top();
            lower.pop();
            p_mid = lower.top();
            lower.pop();
            p_right = lower.top();
            lower.push(p_mid);
            lower.push(p_left);

            while (upper.size() > 2 &&
                   TurnRight(p_right, p_mid, p_left) == false) {
                lower.pop();
                lower.pop();
                lower.push(p_left);

                p_left = lower.top();
                lower.pop();
                p_mid = lower.top();
                lower.pop();
                p_right = lower.top();
                lower.push(p_mid);
                lower.push(p_left);
            }
        }

        std::cout << "OK\n";

        vector<vector<int>> upper_vec(upper.size());
        for (int i = 0; i < upper_vec.size(); i++) {
            upper_vec[i] = upper.top();
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
        for (vector<int>& p : lower_vec)
            upper_vec.push_back(p);
        return upper_vec;
    }
};
