class Solution {
public:
    int threeSumMulti(vector<int>& A, int target) {
        long int res = 0, modNum = 1000000007;
        unordered_map<long int, long int> eleNum;
        sort(A.begin(), A.end());
        for (auto x : A)
            eleNum[x]++;
        
        for (int i = 0; i <= A.size() - 3; i++){
            if (i > 0 && A[i] == A[i - 1])
                continue;

            for (int j = i + 1; j <= A.size() - 2; j++){
                if (j > i + 1 && A[j] == A[j - 1])
                    continue;
                for (int k = j + 1; k <= A.size() - 1; k++){
                    if (k > j + 1 && A[k] == A[k - 1])
                        continue;

                    if (A[i] + A[j] + A[k] < target)
                        continue;
                    else if (A[i] + A[j] + A[k] == target){
                        if (A[i] != A[j] && A[j] != A[k]){
                            res += eleNum[A[i]] * eleNum[A[j]] * eleNum[A[k]];
                            res %= modNum;
                        }
                        else if (A[i] == A[j] && A[j] != A[k]){
                            res += eleNum[A[i]] * (eleNum[A[i]] - 1) * eleNum[A[k]] / 2;
                            res %= modNum;
                        }
                        else if (A[i] != A[j] && A[j] == A[k]){
                            res += eleNum[A[i]] * eleNum[A[j]] * (eleNum[A[j]] - 1) / 2;
                            res %= modNum;
                        }
                        else{   
                            res += eleNum[A[i]] * (eleNum[A[i]] - 1) * (eleNum[A[i]] - 2) / 6;
                            res %= modNum;
                        }
                        break;
                    }
                    else
                        break;
                }
            }
        }
        return res;
    }
};
