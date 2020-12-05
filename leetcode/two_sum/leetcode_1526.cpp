// The intersesting thing is to understand that the generous case is no difference with the
// sorted case in terms of the operation number. At each level, the added columns are actually 
// definite, for general unsorted case, it just has a different specific indecs order, nothing
// new for operations needed.

class Solution {
public:
    int minNumberOperations(vector<int>& target) {
        int res = target[0];
        for (int i  = 1; i < target.size(); i++)
            if (target[i] > target[i - 1])
                res += target[i] - target[i - 1];
        return res;

        /* It is cool for unsing this algorithm similar to bfs, but math is more powerful.
        int minNum = *min_element(target.begin(), target.end()),
            maxNum = *max_element(target.begin(), target.end()),
            res = minNum;
        queue<int> validIdxQueue;
        for (int i = 0; i < target.size(); i++)
            if (target[i] > minNum) 
                validIdxQueue.push(i);
            
        for (int i = minNum + 1; i <= maxNum; i++) {
            int validSize = validIdxQueue.size(), subNum = 0, curIdx, preIdx = -10;
            for (int j = 0; j < validSize; j++) {
                curIdx = validIdxQueue.front();
                validIdxQueue.pop();
                if (preIdx != curIdx - 1)
                    subNum++;
                if (target[curIdx] > i)
                    validIdxQueue.push(curIdx);
                preIdx = curIdx;
            }
            res += subNum;
        }
        return res;
        */
    }
};
