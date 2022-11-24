class Solution {
public:
    vector<int> hour_digit{1, 2, 4, 8}, 
                min_digit{1, 2, 4, 8, 16, 32};
    vector<vector<int>> hour_num,
                        min_num;
    bool initialized = false;

    void init() {
        hour_num = vector<vector<int>>(4, vector<int>());
        min_num = vector<vector<int>>(6, vector<int>());

        hour_num[0].push_back(0);
        for (int digit_num = 1; digit_num <= 3; digit_num++) {
            int turned_num = 0, sum = 0;
            dfs(hour_digit, 0, digit_num, turned_num, sum, hour_num[digit_num]);
        }
        min_num[0].push_back(0);
        for (int digit_num = 1; digit_num <= 5; digit_num++) {
            int turned_num = 0, sum = 0;
            dfs(min_digit, 0, digit_num, turned_num, sum, min_num[digit_num]);
        } 
    }


    void dfs(vector<int>& digits, int cur_pos, int digit_num, int& turned_num, int& sum, vector<int>& res) {
        if (turned_num == digit_num) {
            res.push_back(sum);
            return;
        }
        
        for (int i = cur_pos; i < digits.size(); i++) { 
            sum += digits[i];
            turned_num++;
            dfs(digits, i + 1, digit_num, turned_num, sum, res);
            sum -= digits[i];
            turned_num--;   
        }
    }

    
    vector<string> readBinaryWatch(int turnedOn) {
        if (turnedOn > 8)
            return {};  

        if (initialized == false) {
            init();
            initialized = true;
        }
        vector<string> res;
        for (int hour_on = 0; hour_on <= 3 && hour_on <= turnedOn; hour_on++) {
            int min_on = turnedOn - hour_on;
            if (min_on > 5)
                continue;
            generateStringRes(hour_num[hour_on], min_num[min_on], res);
        }

        return res;
    }


    void generateStringRes(vector<int>& hour, vector<int>& min, vector<string>& res) {
        for (int i = 0; i < hour.size(); i++) {
            if (hour[i] > 11)
                break;
            string hour_str = to_string(hour[i]);

            for (int j = 0; j < min.size(); j++) {
                if (min[j] > 59)
                    break;
                string min_str = to_string(min[j]);
                if (min[j] < 10)
                    min_str = "0" + min_str;
                string res_str = hour_str + ":" + min_str;
                res.push_back(res_str);
            }
        }
    }
};
