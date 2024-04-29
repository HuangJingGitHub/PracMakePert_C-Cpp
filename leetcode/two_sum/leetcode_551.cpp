class Solution {
public:
    bool checkRecord(string s) {
        int num_of_A = 0, num_of_consectutive_L = 0;
        bool is_enough_L = false;

        for (int i = 0; i < s.size(); ) {
            if (s[i] == 'A') {
                num_of_A++;
                i++;
            }
            else if (s[i] == 'L') {
                num_of_consectutive_L = 0;
                while (s[i] == 'L' && i < s.size()) {
                    num_of_consectutive_L++;
                    i++;
                }
                if (num_of_consectutive_L >= 3)
                    is_enough_L = true;
            }
            else 
                i++;
        }

        return num_of_A < 2 && is_enough_L == false;
    }
};
