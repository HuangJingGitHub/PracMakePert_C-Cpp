/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    bool hasCycle(ListNode *head) {
        // Sol 1: set
        /*
        unordered_set<ListNode*> visited;
        while (head) {
            if (visited.find(head) != visited.end())
                return true;
            else
                visited.insert(head);
            head = head->next;
        }
        return false; */
        
        // Sol 2: fast-slow pointers
        if (!head || !head->next)
            return false;
        ListNode *slowPt = head, *fastPt = head->next;
        while (slowPt != fastPt) {
            if (!fastPt || !fastPt->next)
                return false;
            slowPt = slowPt->next;
            fastPt = fastPt->next->next;
        } 
        return true;
    }
};
