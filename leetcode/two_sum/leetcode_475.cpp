class Solution {
public:
    int findRadius(vector<int>& houses, vector<int>& heaters) {
        sort(houses.begin(), houses.end());
        sort(heaters.begin(), heaters.end());

        int res = 0;
        vector<vector<double>> border(heaters.size(), vector<double>(2, 0.0));

        border[0][0] = 1;
        border[0][1] = heaters.size() == 1 ? 1e9 : heaters[0] + ((double)heaters[1] - heaters[0]) / 2;

        for (int i = 1; i < heaters.size() - 1; i++) {
            border[i][0] = heaters[i - 1] + ((double)heaters[i] - heaters[i - 1]) / 2;
            border[i][1] = heaters[i] + ((double)heaters[i + 1] - heaters[i]) / 2;
        }

        if (heaters.size() > 1) {
            border.back()[0] = heaters[heaters.size() - 2] + ((double)heaters.back() - heaters[heaters.size() - 2]) / 2;
            border.back()[1] = 1e9;
        }

        int heater_idx = 0;
        for (int i = 0; i < houses.size(); i++) {
            while (!(houses[i] >= border[heater_idx][0] && houses[i] <= border[heater_idx][1]))
                heater_idx++;
            res = max(res, abs(houses[i] - heaters[heater_idx]));
        }

        return res;
    }
};
