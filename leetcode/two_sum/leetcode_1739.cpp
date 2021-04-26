class Solution {
public:
    int minimumBoxes(int n) {
        int k  = 1, sum = 0, res = 0;
        // 1, 3, 6, 10, ... every level
        while (sum + k * (k + 1) / 2 < n) {
            sum += k * (k + 1) / 2;
            k++;
        }
        
        k--; 
        res = k * (k + 1) / 2;
        k = 1;
        // construct a second pile of box
        while(sum < n) {
            sum += k;
            k++;
            res++;
        }
        return res;
    }
};
