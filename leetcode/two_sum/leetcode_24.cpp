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
    ListNode* swapPairs(ListNode* head) {
        if (head == NULL || head->next == NULL)
            return head;
        ListNode *temp, *originalHead = head->next; 
        for (;head != NULL && head->next != NULL;){
            temp = head->next; 
            cout << temp->val << endl;
            head->next = head->next->next;
            temp->next = head;
            head = head->next;
            for (ListNode* tp = originalHead; tp != NULL; tp = tp->next)
                cout << tp->val << " ";
            cout << endl;
        }
        return originalHead;      
    }
};
