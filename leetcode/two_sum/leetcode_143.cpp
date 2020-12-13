/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    void reorderList(ListNode* head) {
        if (!head || !head->next || !head->next->next)
            return;

        ListNode *headPt = head, *headNextPt, *tailPt;
        while (headPt->next && headPt->next->next) {
            tailPt = headPt;
            headNextPt = headPt->next;
            while (tailPt->next->next)
                tailPt = tailPt->next;
            headPt->next = tailPt->next;
            tailPt->next->next = headNextPt;
            tailPt->next = NULL;
            headPt = headNextPt;
        }
    }
};

// store nodes
class Solution {
public:
    void reorderList(ListNode* head) {
        if (!head || !head->next || !head->next->next)
            return;

        vector<ListNode*> nodeVec;
        while (head) {
            nodeVec.push_back(head);
            head = head->next;
        }
        int left = 0, right = nodeVec.size() - 1;
        for (; left < right - 1; left++, right--) {
            ListNode* nextPt = nodeVec[left]->next;
            nodeVec[left]->next = nodeVec[right];
            nodeVec[right]->next = nextPt;
        }
        if (left == right) // odd node number
            nodeVec[left]->next = NULL;
        else // even node number
            nodeVec[right]->next = NULL;
    }
};
