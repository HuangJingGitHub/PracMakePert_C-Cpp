// This algorithm will exceed the time limit. In the worest case, this implementation is O(kN). (Just consider a decreasing array for instance)
class Solution {
public:
    vector<int> maxSlidingWindow(vector<int>& nums, int k) {
        if (k == 1)
            return nums;

        vector<int> res;
        int left = 0, right = 0, curMax = nums[0], curMaxIdx = 0;
        priority_queue<pair<int, int>> curMaxToRight;

        for (int i = 0; i < k; i++)
            if (nums[i] >= curMax) {
                curMax = nums[i];
                curMaxIdx = i;
            }
        for (int i = curMaxIdx + 1; i < k; i++)
            curMaxToRight.push(pair<int, int>(nums[i], i));

        res.push_back(curMax);
        for (left = 1, right = k; right < nums.size(); left++, right++) {
            if (nums[right] >= curMax) {
                curMax = nums[right];
                curMaxIdx = right;
                curMaxToRight = priority_queue<pair<int, int>>();
            }
            else {
                if (curMaxIdx >= left)
                    curMaxToRight.push(pair<int, int>(nums[right], right));
                else {
                    curMaxToRight.push(pair<int, int>(nums[right], right));
                    curMax = curMaxToRight.top().first;
                    curMaxIdx = curMaxToRight.top().second;
                    curMaxToRight = priority_queue<pair<int, int>>();
                    for (int i = curMaxIdx + 1; i <= right; i++)
                        curMaxToRight.push(pair<int, int>(nums[i], i));
                }
            }
            res.push_back(curMax);
        }
        return res;
    }
};


// Good for monotonic queue
class MonotonicQueue {
private:
    deque<int> data;
public:
    void push(int n) {
        while (!data.empty() && data.back() < n)
            data.pop_back();
        data.push_back(n);
    }

    int max() {
        return data.front();
    }

    void pop(int n) {
        if (!data.empty() && data.front() == n) 
            data.pop_front();
    }
};

class Solution {
public:
    vector<int> maxSlidingWindow(vector<int>& nums, int k) {
        if (k == 1)
            return nums;
        
        vector<int> res;
        MonotonicQueue window;

        int i = 0;
        while (i < nums.size()) {
            if (i < k - 1)
                window.push(nums[i]);
            else {
                window.push(nums[i]);
                res.push_back(window.max());
                window.pop(nums[i - k + 1]);
            }
            i++;
        }
        return res;
    }
};
