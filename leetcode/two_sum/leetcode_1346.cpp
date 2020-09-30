class Solution {
public:
    bool checkIfExist(vector<int>& arr) {
        unordered_set<int> halfDoubleNum;

        for (int i = 0; i < arr.size(); i++){
            if (halfDoubleNum.find(arr[i]) != halfDoubleNum.end())
                return true;
            
            halfDoubleNum.emplace(2 * arr[i]);
            if (arr[i] % 2 == 0)
                halfDoubleNum.emplace(arr[i] / 2);    
        }

        return false;
    }
};

// Slightly better as relatively a smaller set scale is needed.
class Solution {
public:
    bool checkIfExist(vector<int>& arr) {
    unordered_set<int> arrSet;
    for (int x : arr){
        if (arrSet.find(2 * x) != arrSet.end() || (x % 2 == 0 && arrSet.find(x / 2) != arrSet.end()))
            return true;
        arrSet.emplace(x);
    }
    return false;
    }
};
