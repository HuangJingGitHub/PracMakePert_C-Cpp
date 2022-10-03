class Solution {
public:
    string reformatDate(string date) {
        string day, month, year, res;
        vector<int> space_index;
        vector<string> month_str = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

        for (int i = 0; i < date.size(); i++)
            if (date[i] == ' ')
                space_index.push_back(i);
        day = date.substr(0, space_index[0]);
        month = date.substr(space_index[0] + 1, space_index[1] - space_index[0] - 1);
        year = date.substr(space_index[1] + 1);

        while (day.back() < '0' || day.back() > '9')
            day.pop_back();
        int day_num = stoi(day);

        int month_num = 0;
        for (int i = 1; i <= 12; i++) {
            if (month_str[i] == month) {
                month_num = i;
                break;
            }
        }

        res += year;
        res += "-";
        if (month_num <= 9)
            res += "0";
        res += to_string(month_num);
        res += "-";
        if (day_num <= 9)
            res += "0";
        res += to_string(day_num);

        return res;
    }
};
