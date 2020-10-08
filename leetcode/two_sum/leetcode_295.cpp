/** Main idea is to maintain a max heap for small half elements and a min heap for larger half elements. Deal with their amount relationship 
 * Look the favorited solution for reference. 
 */
class MedianFinder {
private:
    int dataCount;
    priority_queue<int> leftMaxHeap;
    priority_queue<int, vector<int>, greater<int>> rightMinHeap;

public:
    MedianFinder() {
        dataCount = 0;
    }
    
    void addNum(int num) {
        dataCount++;
        if ((dataCount & 1) == 1){   // Note that bitwise and & has a lower precedence than >, <, ==, <<, etc. So parentheses are needed.
            leftMaxHeap.push(num);
            if (!rightMinHeap.empty()){
                leftMaxHeap.push(rightMinHeap.top());
                rightMinHeap.pop();

                rightMinHeap.push(leftMaxHeap.top());
                leftMaxHeap.pop();
            }
        }
        else{
            leftMaxHeap.push(num);
            rightMinHeap.push(leftMaxHeap.top());
            leftMaxHeap.pop();
        }
    }
    
    double findMedian() {
        if ((dataCount & 1) == 1)
            return leftMaxHeap.top();
        else
            return leftMaxHeap.top() + (rightMinHeap.top() - leftMaxHeap.top()) / 2.0;  // pay attention return double value.
    }
};

/**
 * Your MedianFinder object will be instantiated and called as such:
 * MedianFinder* obj = new MedianFinder();
 * obj->addNum(num);
 * double param_2 = obj->findMedian();
 */
