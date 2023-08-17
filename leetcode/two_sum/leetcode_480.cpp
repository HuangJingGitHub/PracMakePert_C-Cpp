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

// High time complexity
class Solution {
public:
    ListNode* list_head;
    int window_size;

    vector<double> medianSlidingWindow(vector<int>& nums, int k) {
        vector<double> res(nums.size() - k + 1);

        initList(nums, k);
        window_size = k;
        res[0] = getListMedian();

        for (int window_end = k; window_end < nums.size(); window_end++) {
            updateList(nums[window_end - k], nums[window_end]);
            res[window_end - k + 1] = getListMedian();
        }
        return res;
    }

    void initList(vector<int>& nums, int k) {
        vector<int> window(k);
        for (int i = 0; i < k; i++)
            window[i] = nums[i];
        sort(window.begin(), window.end());

        list_head = new ListNode(window[0]);
        ListNode* node_ptr = list_head;
        for (int i = 1; i < k; i++) {
            node_ptr->next = new ListNode(window[i]);
            node_ptr = node_ptr->next;
        }      
    }

    double getListMedian() {
        if (window_size == 1)
            return (double) list_head->val;

        ListNode* node_ptr = list_head;
        node_ptr = list_head;
        int cnt = 0;
        while (cnt++ < window_size / 2 - 1)
            node_ptr = node_ptr->next;
        if (window_size % 2 == 0) 
            return ((double)node_ptr->val + (double)node_ptr->next->val) / 2.0;
        return (double) node_ptr->next->val;
    }   

    void updateList(int removed_num, int added_num) {
        if (window_size == 1) {
            list_head->val = added_num;
            return;
        }

        ListNode *pre_ptr = list_head, *node_ptr = list_head->next;
        if (list_head->val == removed_num)
            list_head = list_head->next;
        else {
            while (node_ptr != nullptr) {
                if (node_ptr->val == removed_num) {
                    pre_ptr->next = node_ptr->next;
                    break;
                }
                pre_ptr = node_ptr;
                node_ptr = node_ptr->next;
            }
        }


        ListNode* new_node = new ListNode(added_num);
        if (added_num <= list_head->val) {
            new_node->next = list_head;
            list_head = new_node;
            return;
        }
        
        pre_ptr = list_head;
        node_ptr = list_head->next;
        while (node_ptr != nullptr) {
            if (pre_ptr->val <= added_num && added_num <= node_ptr->val) {
                pre_ptr->next = new_node;
                new_node->next = node_ptr;
                return;
            }
            pre_ptr = node_ptr;
            node_ptr = node_ptr->next;
        }

        pre_ptr->next = new_node;
    }
};


// maintain max-heap, min-heap
class Solution {
public:
    priority_queue<int> small;
    priority_queue<int, vector<int>, greater<int>> big;
    unordered_map<int, int> mp;

    double get(int& k){
        if (k % 2 == 1) return small.top();
        else return 
            ((long long)small.top() + big.top()) * 0.5;
    }

    vector<double> medianSlidingWindow(vector<int>& nums, int k) {
        for (int i = 0; i < k; i++) {
            small.push(nums[i]);
        }
        for (int i = 0; i < k / 2; i++) {
            big.push(small.top()); 
            small.pop();
        }

        vector<double> ans{get(k)};
        for(int i = k; i < nums.size(); i++){
            int balance = 0;
            int l = nums[i - k];
            mp[l]++;
            if (!small.empty() && l <= small.top()) 
                balance--;
            else 
                balance++;
            if (!small.empty() && nums[i] <= small.top()) {
                small.push(nums[i]);
                balance++;
            }
            else {
                big.push(nums[i]);
                balance--;
            }
            
            while (!small.empty() && mp[small.top()] > 0){
                mp[small.top()]--;
                small.pop();
            }
            while (!big.empty() && mp[big.top()] > 0){
                mp[big.top()]--;
                big.pop();
            }

            if (balance > 0){
                big.push(small.top());
                small.pop();
            }
            if (balance < 0){
                small.push(big.top());
                big.pop();
            }
            while (!big.empty() && mp[big.top()] > 0) {
                mp[big.top()]--;
                big.pop();
            }
            while (!small.empty() && mp[small.top()] > 0) {
                mp[small.top()]--;
                small.pop();
            }
            
            ans.push_back(get(k));
            
        }
        return ans;
    }
};
