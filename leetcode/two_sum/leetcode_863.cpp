/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
class Solution {
public:
    vector<int> distanceK(TreeNode* root, TreeNode* target, int k) {
        map<int, vector<int>> adj_list;
        queue<TreeNode*> node_queue;
        node_queue.push(root);

        while (!node_queue.empty()) {
            TreeNode* node = node_queue.front();
            node_queue.pop();

            if (node->left) {
                adj_list[node->val].push_back(node->left->val);
                adj_list[node->left->val].push_back(node->val);
                node_queue.push(node->left);
            }
            if (node->right) {
                adj_list[node->val].push_back(node->right->val);
                adj_list[node->right->val].push_back(node->val);
                node_queue.push(node->right);
            }
        }

        map<int, bool> visited;
        queue<int> dist_queue;
        dist_queue.push(target->val);
        visited[target->val] = true;

        for (int dist = 0; dist < k; dist++) {
            int size = dist_queue.size();
            for (int i = 0; i < size; i++) {
                int num = dist_queue.front();
                dist_queue.pop();

                for (int key : adj_list[num]) {
                    if (!visited[key]) {
                        dist_queue.push(key);
                        visited[key] = true;
                    }
                }
            }
        }

        vector<int> res;
        while (!dist_queue.empty()) {
            res.push_back(dist_queue.front());
            dist_queue.pop();
        }
        return res;
    }
};
