class MedianFinder {
private:
    priority_queue<int> leftMaxHeap;
    priority_queue<int, vector<int>, greater<int>> rightMinHeap;
    int dataCount;

public:
    /** initialize your data structure here. */
    MedianFinder() {
        dataCount = 0;
    }
    
    void addNum(int num) {
        dataCount++;
        if (dataCount % 2 != 0)
            leftMaxHeap.push(num);
        else
            rightMinHeap.push(num);
    }
    
    double findMedian() {
        if (dataCount % 2 != 0)
            return leftMaxHeap.top();
        else
            return leftMaxHeap.top() + (rightMinHeap.top() - leftMaxHeap.top()) / 2.0;
    }
};

/**
 * Your MedianFinder object will be instantiated and called as such:
 * MedianFinder* obj = new MedianFinder();
 * obj->addNum(num);
 * double param_2 = obj->findMedian();
 */
