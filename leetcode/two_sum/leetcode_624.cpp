class Solution {
public:
    int maxDistance(vector<vector<int>>& arrays) {
        vector<pair<int, int>> min_arr, max_arr;
        for (int i = 0; i < arrays.size(); i++) {
            min_arr.push_back(make_pair(arrays[i].front(), i));
            max_arr.push_back(make_pair(arrays[i].back(), i));
        }
        sort(min_arr.begin(), min_arr.end());
        sort(max_arr.begin(), max_arr.end());

        int min_idx = 0, max_idx = arrays.size() - 1;
        while (min_arr[min_idx].second == max_arr[max_idx].second) {
            if (min_arr[min_idx + 1].first - min_arr[min_idx].first < max_arr[max_idx].first - max_arr[max_idx - 1].first)
                min_idx++;
            else
                max_idx--;
        }
        return max_arr[max_idx].first - min_arr[min_idx].first;
    }
};
