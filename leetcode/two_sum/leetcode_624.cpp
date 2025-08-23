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

class Solution {
public:
    int maxDistance(vector<vector<int>>& arrays) {
      int min_val = arrays[0].front(), max_val = arrays[0].back(), res = 0;

      for (int i = 1; i < arrays.size(); i++) {
        res = max({res, abs(max_val - arrays[i].front()), 
                    abs(arrays[i].back() - min_val)});
        max_val = max(max_val, arrays[i].back());
        min_val = min(min_val, arrays[i].front());
      }  

      return res;
    }
};
