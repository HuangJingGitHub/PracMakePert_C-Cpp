// Bulky solution
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
    vector<int> nextLargerNodes(ListNode* head) {
        vector<int> res;
        for (ListNode* ptr = head; ptr != NULL; ptr = ptr->next){
            ListNode* slidingPtr = ptr->next;           
            for (; slidingPtr != NULL; slidingPtr = slidingPtr->next){
                if (slidingPtr->val > ptr->val){
                    res.push_back(slidingPtr->val);
                    break;
                }
            }
            if (!slidingPtr)
                res.push_back(0);
        }

        return res;
    }
};


// Bulky solution with little optimization
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
    vector<int> nextLargerNodes(ListNode* head) {
        vector<int> res;
        int supremum = INT_MAX;
        for (ListNode* ptr = head; ptr != NULL; ptr = ptr->next){
            ListNode* slidingPtr = ptr->next;
            if (ptr->val >= supremum){
                res.push_back(0);
                continue;
            }
            
            for (; slidingPtr != NULL; slidingPtr = slidingPtr->next){
                if (slidingPtr->val > ptr->val){
                    res.push_back(slidingPtr->val);
                    break;
                }
            }
            if (!slidingPtr){
                res.push_back(0);
                supremum = ptr->val < supremum ? ptr->val : supremum;
            }
        }

        return res;
    }
};
