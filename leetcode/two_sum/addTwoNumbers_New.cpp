#include <iostream>
using namespace std;

  struct ListNode {
      int val;
      ListNode *next;
      ListNode(int x) : val(x), next(NULL) {}
  };


void printList(ListNode* list)
{
	ListNode *print_ptr = list;
    cout << print_ptr->val << " ";
    for ( ; print_ptr->next != NULL; )
    {   
		print_ptr = print_ptr->next;
        cout << print_ptr->val << " ";
    }
	print_ptr = NULL;
}


class Solution {
public:
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) 
	{
		static ListNode *res;
		ListNode *preNode = NULL;
		bool carry = false, end_carry = false;
		int node_sum;

		for (int counter = 1; l1 != NULL || l2 != NULL || end_carry; )
		{
			if (l1 != NULL && l2 != NULL)
				node_sum = l1->val + l2->val;
			else if (l1 != NULL && l2 == NULL)
				node_sum = l1->val;
			else if (l1 == NULL && l2 != NULL)
				node_sum = l2->val;
	
			node_sum = (carry) ? node_sum + 1 : node_sum;
			if (node_sum >= 10)
			{
				node_sum = node_sum - 10;
				carry = true;
			}
			else
				carry = false;
			
			ListNode *newNode = new ListNode(0);
			if (counter == 1)
			{
				res = newNode;
				preNode = newNode;
			}
			if (counter != 1)
			{
				preNode->next = newNode;
				preNode = newNode;
			}
			
			newNode->val = node_sum;
			if (l1 != NULL)
				l1 = (l1->next == NULL) ? NULL : l1->next;
			if (l2 != NULL)
				l2 = (l2->next == NULL) ? NULL : l2->next;
			if (l1 == NULL && l2 == NULL && carry)
				end_carry = true;
			counter++;
			cout << "test: " << newNode->val << endl;	
		}
		printList(res);
		return res;
    }
};


int main()
{
    ListNode *p1,  *p2, *p3;
    ListNode L1(3), L2(6), L3(8);
    p1 = &L1;
    L1.next = &L2;
	L2.next = &L3;
    p2 = &L3;
    Solution sol;

    //printList(sol.addTwoNumbers(p1, p2));
    // cout << endl;
    printList(p1);
    cout << endl;
    printList(p2);
    cout << endl;
	p3 = sol.addTwoNumbers(p1, p2);
}
