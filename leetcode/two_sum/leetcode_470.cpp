// The rand7() API is already defined for you.
// int rand7();
// @return a random integer in the range 1 to 7

class Solution {
public:
    int rand10() {
        int first_rand = 10, second_rand = 4, res = 0;
       
        while (first_rand > 5)
            first_rand = rand7();
        while (second_rand == 4)
            second_rand = rand7();
        
        if (second_rand < 4)
            res = first_rand;
        else
            res = first_rand + 5;
        return res;
    }
};
