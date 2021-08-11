class Solution {
public:
    int nthSuperUglyNumber(int n, vector<int>& primes) {
        if (n == 1)
            return 1;

        long long int res = 1;
        priority_queue<long long int, vector<long long int>, greater<long long int>> minHeap;
        minHeap.push(1);

        for (int i = 1; i < n; i++) {
            res = minHeap.top();
            while (!minHeap.empty() && minHeap.top() == res)
                minHeap.pop();
            for (int& num : primes)
                minHeap.push(res * num);
        }
        return minHeap.top();
    }
};
