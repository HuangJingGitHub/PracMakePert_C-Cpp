class Solution {
public:
    int findMinMoves(vector<int>& machines) {
        int total = accumulate(machines.begin(), machines.end(), 0);
        int n = machines.size();

        if (total % n != 0)
            return -1;
        
        int avg = total / n, res = 0, sum = 0;
        for (int dresses : machines) {
            dresses -= avg;
            sum += dresses;
            res = max(res, max(abs(sum), dresses));
        }
        return res;
    }
};
