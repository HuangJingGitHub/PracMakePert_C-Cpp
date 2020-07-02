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
    ListNode* partition(ListNode* head, int x) {
        /*ListNode *pt = new ListNode(0);
        pt->next = head;
        head = pt;
        
        ListNode *smallPt = pt, *rightPt = pt, *tmpSmall, *tmpRight;
        while (rightPt->next){
            while (!rightPt->next && rightPt->next->val >= x)
                rightPt = rightPt->next;
            if (rightPt->next == NULL)
                break;
            
            tmpSmall = smallPt->next;
            tmpRight = rightPt->next;
            rightPt->next = rightPt->next->next;
            rightPt = rightPt->next;
            smallPt->next = tmpRight;
            tmpRight->next = (tmpSmall == tmpRight) ? tmpRight->next : tmpSmall;
            smallPt = smallPt->next;

            if (rightPt == NULL)
                break;
        }
        return head->next; */
        ListNode *node1 = new ListNode(0), *node2 = new ListNode(0), *pt1 = node1, *pt2 = node2;
        while (head){
            cout << head->val << " ";
            if (head->val < x){
                pt1->next = head;
                pt1 = pt1->next;
            }
            else{
                pt2->next = head;
                pt2 = pt2->next;
            }
            head = head->next;
        }
        pt2->next = NULL;  // Do not forget this important step.
        pt1->next = node2->next; 
        return node1->next;
    }
};
