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
    ListNode *classHead_;
    Solution(ListNode* head) {
        classHead_ = head;
        srand(time(0));
    }
    
    int getRandom() {
        ListNode *head = classHead_;
        int res = head->val;
        int cnt = 1;

        while (head != nullptr) {
            int temp = rand() % cnt;
            if (temp + 1 == cnt)
                res = head->val;
            
            cnt++;
            head = head->next;
        }

        return res;
    }
};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(head);
 * int param_1 = obj->getRandom();
 */