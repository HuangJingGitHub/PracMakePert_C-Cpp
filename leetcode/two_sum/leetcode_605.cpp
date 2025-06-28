class Solution {
public:
    bool canPlaceFlowers(vector<int>& flowerbed, int n) {
        int place_num = 0, cnt = 1;
        for (int i = 0; i < flowerbed.size(); i++) {
            if (flowerbed[i] == 1) {
                if (cnt == 0)
                    continue;
                else {
                    place_num += (cnt - 1) / 2;
                    cnt = 0;
                }
            }
            else
                cnt++;
        }
        cnt++;
        place_num += (cnt - 1) / 2;
        return place_num >= n;
    }
};

