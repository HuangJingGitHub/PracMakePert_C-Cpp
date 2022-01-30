class Solution {
public:
    int calculateMinimumHP(vector<vector<int>>& dungeon) {
        if (dungeon.empty() || dungeon[0].empty())
            return 0;
        int rowNum = dungeon.size(), colNum = dungeon[0].size(), healthRequirement = 0;
        dungeon.back().back() = max(1, 1 - dungeon.back().back());  // Pay attention if the last grid is positive like 100, the health needs to be 1, not 1 - 100
        for (int i = colNum - 2; i >= 0; i--) {
            if (dungeon.back()[i] >= dungeon.back()[i + 1])
                dungeon.back()[i] = 1;
            else
                dungeon.back()[i] = dungeon.back()[i + 1] - dungeon.back()[i];
        }
        for (int i  = rowNum - 2; i >= 0; i--) {
            if (dungeon[i].back() >= dungeon[i + 1].back())
                dungeon[i].back() = 1;
            else
                dungeon[i].back() = dungeon[i + 1].back() - dungeon[i].back();
        }
        for (int row = rowNum - 2; row >= 0; row--)
            for (int col = colNum - 2; col >= 0; col--) {
                if (dungeon[row][col] >= min(dungeon[row][col + 1], dungeon[row + 1][col]))
                    dungeon[row][col] = 1;
                else
                    dungeon[row][col] = min(dungeon[row][col + 1], dungeon[row + 1][col]) - dungeon[row][col];
            }
        return dungeon[0][0]; 
    }
};


// More concise
class Solution {
public:
    int calculateMinimumHP(vector<vector<int>>& dungeon) {
        int rowNum = dungeon.size(), colNum = dungeon[0].size();

        dungeon.back().back() = max(1 - dungeon.back().back(), 1);
        for (int row = rowNum - 2; row >= 0; row--)
            dungeon[row].back() =  max(dungeon[row + 1].back() - dungeon[row].back(), 1);
        for (int col = colNum - 2; col >= 0; col--)
            dungeon.back()[col] = max(dungeon.back()[col + 1] - dungeon.back()[col], 1);
        
        for (int row = rowNum - 2; row >= 0; row--)
            for (int col = colNum - 2; col >= 0; col--)
                dungeon[row][col] = max(min(dungeon[row][col + 1], dungeon[row + 1][col]) - dungeon[row][col], 1);

        return dungeon[0][0];
    }
};
