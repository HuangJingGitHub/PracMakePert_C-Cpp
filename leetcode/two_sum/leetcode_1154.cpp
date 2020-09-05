class Solution {
public:
    int dayOfYear(string date) {
        vector<int> monthDays{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int years = 0, months = 0, days = 0, res = 0, i = 0;
        for (; i < 4; i++)
            years = years*10 + date[i] - 48;
        for (i = 5; i < 7; i++)
            months = months*10 + date[i] - 48;
        for (i = 8; i < 10; i++)
            days = days*10 + date[i] - 48;
        
        if (years % 4 == 0 && years % 100 != 0 || years % 400 == 0)
            monthDays[1] = 29;
        for (i = 0; i < months-1; i++)
            res += monthDays[i];
        
        return res + days;
    }
};
