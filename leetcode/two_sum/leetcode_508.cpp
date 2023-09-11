l/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode() : val(0), left(nullptr), right(nullptr) {}
 *     TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
 *     TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
 * };
 */
class Solution {
public:
unordered_map<int, int> sum_frequency;
int max_frequency = 0;

    vector<int> findFrequentTreeSum(TreeNode* root) {
      vector<int> res;
      treeSum(root);

      for (auto it = sum_frequency.begin(); it != sum_frequency.end(); it++) {
        if (it->second == max_frequency)
            res.push_back(it->first);
      }
      return res;
    }

    int treeSum(TreeNode* root) {
      if (root == nullptr)
        return 0;      

      int left_sum = treeSum(root->left);
      int right_sum = treeSum(root->right);
      int subtree_sum = left_sum + right_sum + root->val;
      sum_frequency[subtree_sum]++;
      max_frequency = max(max_frequency, sum_frequency[subtree_sum]);
      return subtree_sum;
    }
};
