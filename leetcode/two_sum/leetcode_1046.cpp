class Solution {
public:
    int lastStoneWeight(vector<int>& stones) {
        priority_queue<int> maxHeap;
        for (int x : stones)
            maxHeap.push(x);
        
        int maxWeight = 0;
        while (!maxHeap.empty()) {
            maxWeight = maxHeap.top();
            maxHeap.pop();
            if (maxHeap.empty())
                return maxWeight;
            else if (maxWeight == maxHeap.top())
                maxHeap.pop();
            else {
                int temp = maxWeight - maxHeap.top();
                maxHeap.pop();     // Ensure pop before push, otherwise what is poped is the updated heap top.
                maxHeap.push(temp);
            }
        }
        return 0;
    }
};
