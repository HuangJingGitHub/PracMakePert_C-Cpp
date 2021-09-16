class Solution {
public:
    int rob(TreeNode* root) {
    vector<int> result = robTree(root);
    return max(result[0], result[1]);
    }

    vector<int> robTree(TreeNode* curNode) {
        if (curNode == NULL)
            return {0, 0};
        vector<int> leftRes = robTree(curNode->left),
                    rightRes = robTree(curNode->right);
        int val1 = curNode->val + leftRes[1] + rightRes[1]; // steal current node
        int val2 = max(leftRes[0], leftRes[1]) + max(rightRes[0], rightRes[1]); 
        return {val1, val2};
    }
};
