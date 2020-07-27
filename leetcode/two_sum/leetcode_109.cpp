/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
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
    TreeNode* sortedListToBST(ListNode* head) {
        if (head == NULL) 
            return NULL;
        return buildBST(head, NULL);
    }

    TreeNode* buildBST(ListNode* start, ListNode* end)
    {
        if (start == end)
            return NULL;
        
        ListNode *fastPt = start, *slowPt = start;
        while(fastPt != end && fastPt->next != end){
            fastPt = fastPt->next->next;
            slowPt = slowPt->next;
        }

        TreeNode* newNode = new TreeNode(slowPt->val);
        newNode->left = buildBST(start, slowPt);
        newNode->right = buildBST(slowPt->next, end);
        return newNode;
    }
};
