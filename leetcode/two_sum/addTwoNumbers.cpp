#include <iostream>
#include <vector>
using namespace std;

  struct ListNode {
      int val;
      ListNode *next;
      ListNode(int x) : val(x), next(NULL) {}
  };


void printList(ListNode* list)
{
    cout << list->val << " ";
    for ( ; list->next != NULL; )
    {   list = list->next;
        cout << list->val << " ";
    }
}


class Solution {
public:
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
        ListNode sum(0);
        ListNode *sum_ptr = &sum, *result = &sum;
        bool carry = false;
        int bit_sum;
        for (; l1->next == NULL && l2->next == NULL; l1 = l1->next, l2 = l2->next)
        {
            if (l1->next == NULL && l2->next == NULL)
                bit_sum = (carry) ? l1->val + l2->val + 1 : l1->val + l2->val;
            else if (l1->next == NULL)
                bit_sum = (carry) ? l2->val + 1 : l2->val;
            else if (l2->next == NULL)
                bit_sum = (carry) ? l1->val + 1 : l1->val;
            if (bit_sum >= 10)
            {
                carry = true;
                sum_ptr->val = bit_sum - 10;
            }
            else
            {
                sum_ptr->val = bit_sum;
            }
            ListNode newSum(0);
            sum_ptr->next = &newSum;
            sum_ptr = &newSum;
        }

        return result;
    }
};


int main()
{
    cout << "H" << endl;
    ListNode *p1,  *p2;
    ListNode L1(3), L2(5);
    p1 = &L1;
    L1.next = &L2;
    p2 = p1;

    Solution sol;

    printList(sol.addTwoNumbers(p1, p2));
    cout << endl;
    printList(p1);
    cout << endl;
    printList(p2);
    cout << endl;
    printList(sol.addTwoNumbers(p1, p2));

}
