class Solution {
public:
    int canCompleteCircuit(vector<int>& gas, vector<int>& cost) {
        int curGas = 0;
        for (int i = 0, j = -1; i < gas.size(); i++){
            if (gas[i] < cost[i])
                continue;
            curGas = gas[i] - cost[i];
            for (j = i + 1; j != i; j++){
                if (j == gas.size()){
                    j = 0;
                    if (j == i)
                        return i;
                }
                curGas = curGas + gas[j] - cost[j];
                if (curGas < 0)
                    break;
            }
            if (j == i)
                return i;
        }
        return -1;
    }
};


// more compact version
class Solution {
public:
    int canCompleteCircuit(vector<int>& gas, vector<int>& cost) {
        int curGas = 0;
        for (int i = 0, j = -1; i < gas.size(); i++){
            if (gas[i] < cost[i])
                continue;
            curGas = gas[i] - cost[i];
            for (j = (i + 1) % gas.size(); j != i; j = ++j % gas.size()){  // Use % as it is a circle
                curGas = curGas + gas[j] - cost[j];
                if (curGas < 0)
                    break;
            }
            if (j == i)
                return i;
        }
        return -1;
    }
};
