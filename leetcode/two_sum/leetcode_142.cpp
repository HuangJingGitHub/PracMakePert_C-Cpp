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
    ListNode *detectCycle(ListNode *head) {
        // set
        /*
        unordered_set<ListNode*> visited;
        while (head) {
            if (visited.find(head) != visited.end())
                return head;
            visited.insert(head);
            head = head->next;
        }  
        return NULL;
        */
        // fast, slow pointers. The mechanism is interesting, refer to the favoriated solurion for detail.
        ListNode *fastPt = head, *slowPt = head;
        while (true) {
            if (!fastPt || !fastPt->next)
                return NULL;
            slowPt = slowPt->next;
            fastPt = fastPt->next->next;
            if (fastPt == slowPt)
                break;
        }
        fastPt = head;
        while (fastPt != slowPt) {
            slowPt = slowPt->next;
            fastPt = fastPt->next;
        }
        return slowPt;
    }
};
