class Solution {
public:
    double mincostToHireWorkers(vector<int>& quality, vector<int>& wage, int K) {
        set<pair<double, int>> unitWageQuality;
        priority_queue<int> KWorkers;
        int qualitySum = 0;
        double res = 0;

        for (int i = 0; i < quality.size(); i++){
            double qualityCost = (double)wage[i] / quality[i];
            unitWageQuality.insert({qualityCost, quality[i]});
        }

        auto itr = unitWageQuality.begin();
        for (int i = 0; i < K - 1; i++, itr++){
            KWorkers.push(itr->second);
            qualitySum += itr->second;
        }

        res = (qualitySum + itr->second) * itr->first;
        if (itr->second < KWorkers.top()){
            qualitySum = qualitySum - KWorkers.top() + itr->second;
            KWorkers.pop();
            KWorkers.push(itr->second);
        }

        for ( itr++; itr != unitWageQuality.end(); itr++){
            double curRes = (qualitySum + itr->second) * itr->first;
            res = (res < curRes) ? res : curRes;
            if (itr->second < KWorkers.top()){
                qualitySum = qualitySum - KWorkers.top() + itr->second;
                KWorkers.pop();
                KWorkers.push(itr->second);
            }             
        }
        // for (auto it = unitWageQuality.begin(); it != unitWageQuality.end(); it++)
        //    cout << "{" << it->first << ", " << it->second << "}\n";
        return res;
    }
};
