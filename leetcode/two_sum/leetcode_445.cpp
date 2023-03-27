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
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
        vector<int> vec1, vec2;
        ListNode *pt1 = l1, *pt2 = l2;
        while (pt1 != nullptr) {
            vec1.push_back(pt1->val);
            pt1 = pt1->next;
        }
        while (pt2 != nullptr) {
            vec2.push_back(pt2->val);
            pt2 = pt2->next;
        }

        int resLen = max(vec1.size(), vec2.size()), idx1 = vec1.size() - 1, idx2 = vec2.size() - 1,
            resIdx = resLen - 1, num1 = 0, num2 = 0, sum = 0, add = 0;
        vector<int> resVec(resLen, 0);
        while (idx1 >= 0 && idx2 >= 0) {
            num1 = vec1[idx1--];
            num2 = vec2[idx2--];
            sum = num1 + num2 + add;
            resVec[resIdx--] = sum % 10;
            add = sum / 10;
        }
        if (idx1 >= 0) {
            while (idx1 >= 0) {
                sum = vec1[idx1--] + add;
                resVec[resIdx--] = sum % 10;
                add = sum / 10;
            }
        }
        else
            while (idx2 >= 0) {
                sum = vec2[idx2--] + add;
                resVec[resIdx--] = sum % 10;
                add = sum / 10;
            }
        
        ListNode *res = new ListNode(), *curPt = res;
        if (add == 1) {
            curPt->next = new ListNode(1);
            curPt = curPt->next;
        }
        for (int i = 0; i < resVec.size(); i++) {
            curPt->next = new ListNode(resVec[i]);
            curPt = curPt->next;
        }

        return res->next;
    }
};
