class Solution {
public:
    bool isSelfCrossing(vector<int>& distance) {
        if (distance.size() < 4)
            return false;
        
        for (int i = 3; i < distance.size(); i++) {
            if (distance[i] >= distance[i - 2] && distance[i - 1] <= distance[i - 3])
                return true;
            else if ( i > 3 && distance[i - 1] == distance[i - 3] && distance[i - 4] + distance[i] >= distance[i - 2])
                return true;
            else if (i > 4 && distance[i - 3] - distance[i - 5] <= distance[i - 1] && distance[i - 1] <= distance[i - 3]
                    && distance[i - 2] - distance[i - 4] <= distance[i] && distance[i] <= distance[i - 2]
                    && distance[i - 2] >= distance[i - 4])
                return true;
        }
        return false;
    }
};
