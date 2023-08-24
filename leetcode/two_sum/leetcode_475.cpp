class Solution {
public:
    int findRadius(vector<int>& houses, vector<int>& heaters) {
        sort(houses.begin(), houses.end());
        sort(heaters.begin(), heaters.end());

        int res = 0;
        int heater_idx = 0;
        for (int i = 0; i < houses.size(); i++) {
            while (heater_idx < heaters.size() && heaters[heater_idx] <= houses[i])
                heater_idx++;
            if (heater_idx == 0)
                res = max(res, heaters[0] - houses[i]);
            else if (heater_idx == heaters.size())
                res = max(res, houses[i] - heaters.back());
            else
                res = max(res, min(heaters[heater_idx] - houses[i], houses[i] - heaters[heater_idx - 1]));
        }

        return res;
    }
};
