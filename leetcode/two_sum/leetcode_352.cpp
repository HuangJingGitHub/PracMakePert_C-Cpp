/*struct ListNode {
    int val;
    ListNode* next;
    ListNode(): val(0), next(nullptr) {}
    ListNode(int x): val(x), next(nullptr) {}
    ListNode(int x, ListNode* nextPt): val(x), next(nextPt) {}
};*/

class SummaryRanges {
public:
    set<int> numSet_;
    ListNode* head_;

    SummaryRanges() {
        head_ = new ListNode(INT_MIN);
    }
    
    void addNum(int val) {
        if (numSet_.find(val) != numSet_.end())
            return;
        
        numSet_.insert(val);
        ListNode* addedNode = new ListNode(val);
        
        ListNode *pt = head_->next, *prePt = head_;
        while (pt != nullptr && pt->val < val) {
            pt = pt->next;
            prePt = prePt->next;
        }
        prePt->next = addedNode;
        addedNode->next = pt;
    }
    
    vector<vector<int>> getIntervals() {
        vector<vector<int>> res;

        ListNode *pt = head_->next;
        int intervalStart = pt->val, preVal = pt->val;
        while (true) {
            pt = pt->next;
            if (pt == nullptr) {
                res.push_back({intervalStart, preVal});
                break;
            }

            if (pt->val == preVal + 1) {
                preVal = pt->val;
                continue;
            }
            else {
                res.push_back({intervalStart, preVal});
                intervalStart = pt->val;
                preVal = pt->val;
            }
        }
        return res;
    }
};

/**
 * Your SummaryRanges object will be instantiated and called as such:
 * SummaryRanges* obj = new SummaryRanges();
 * obj->addNum(val);
 * vector<vector<int>> param_2 = obj->getIntervals();
 */
