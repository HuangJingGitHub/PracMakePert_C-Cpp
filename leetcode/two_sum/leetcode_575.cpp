class Solution {
public:
    int distributeCandies(vector<int>& candyType) {
        set<int> totalType;
        for (int& type : candyType) {
            totalType.insert(type);
        }
        return min(totalType.size(), candyType.size() / 2);
    }
};
