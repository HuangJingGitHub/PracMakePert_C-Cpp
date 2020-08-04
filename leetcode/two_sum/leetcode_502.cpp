// good practice on max_heap, min_heap
class Solution {
public:
    int findMaximizedCapital(int k, int W, vector<int>& Profits, vector<int>& Capital) {
        priority_queue<int> profitMaxHeap;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> capitalMinHeap;

        int len = Capital.size();
        for (int i = 0; i < len; i++){
            capitalMinHeap.emplace(make_pair(Capital[i], Profits[i]));
        }
        // greedy algorithm
        for (int i = 0; i < k; i++){
            while (!capitalMinHeap.empty() && capitalMinHeap.top().first <= W){
                profitMaxHeap.emplace(capitalMinHeap.top().second);
                capitalMinHeap.pop();
            }

            if (profitMaxHeap.empty())
                break;
            W += profitMaxHeap.top();
            profitMaxHeap.pop();
        }

        return W;
    }
};
