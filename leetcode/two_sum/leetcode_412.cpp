class Solution {
public:
    vector<string> fizzBuzz(int n) {
        vector<string> res(n, "");
        
        for (int i = 1; i <= n; i++)
            res[i - 1] = to_string(i);
        
        for (int i = 3; i <= n; i += 3)
            res[i - 1] = "Fizz";
        for (int i = 5; i <= n; i += 5)
            if (res[i - 1] == "Fizz")
                res[i - 1] = "FizzBuzz";
            else
                res[i - 1] = "Buzz";
        
        return res;
    }
};
