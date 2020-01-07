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
    void reverseKNodes(ListNode* head, ListNode* tail){
        ListNode *preNode = head, *currentNode = head->next, *nextNode;
        while (currentNode != tail->next && currentNode != NULL){
            nextNode = currentNode->next;
            currentNode->next = preNode;
            preNode = currentNode;
            currentNode = nextNode;
        }
        head->next = nextNode;
    }

    ListNode* reverseKGroup(ListNode* head, int k) {
        if (head == NULL || head->next == NULL || k <= 1)
            return head;

        ListNode *dummy = new ListNode(0);
        dummy->next = head;
        ListNode *start = head, *end = head, *preGroupTail = dummy, *nextGroupHead;
        int ListNodeNum, initialTime = 0;
        
        while (end != NULL){
            for (ListNodeNum = 0, start = end, initialTime++; ListNodeNum < k-1 && end != NULL; end = end->next, ListNodeNum++);
            nextGroupHead = end->next;
            if (ListNodeNum == k-1){
                reverseKNodes(start, end);
                preGroupTail->next = end;
                preGroupTail = start;
                end = nextGroupHead;
            }       
        }
        return dummy->next;    
    }
};

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
    ListNode* reverseKGroup(ListNode* head, int k) {
        ListNode *dummy = new ListNode(-1), *pre = dummy, *cur = pre;
        dummy->next = head;
        int num = 0;
        while (cur = cur->next) ++num; //链表长度
        while (num >= k) {
            cur = pre->next;
            //依次交换节点元素
            for (int i = 1; i < k; ++i) {
                ListNode *t = cur->next;
                cur->next = t->next;
                t->next = pre->next;
                pre->next = t;
            }
            pre = cur;
            num -= k;
        }
        return dummy->next;
    }
};
