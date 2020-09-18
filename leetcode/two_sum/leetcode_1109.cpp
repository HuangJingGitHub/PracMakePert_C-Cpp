class Solution {
public:
    vector<int> corpFlightBookings(vector<vector<int>>& bookings, int n) {
        /*vector<int> answer(n, 0);
        for (int i = 0; i < bookings.size(); i++){
            for (int idx = bookings[i][0]; idx <= bookings[i][1]; idx++)
                answer[idx-1] += bookings[i].back();
        }
        return anwser;*/ // direct methods will exceed time limit
        
        // Interesting one-loop algorithm
        vector<int> res(n, 0);  
        for (int i = 0; i < bookings.size(); i++){
            res[bookings[i][0] - 1] += bookings[i][2];   // record the increment start position
            if (bookings[i][1] < n)
                res[bookings[i][1]] -= bookings[i][2];   // record the increment end position
        }
        for (int i = 1; i < n; i++)
            res[i] += res[i-1];  // self increment to get the sum
        return res;
    }
};
