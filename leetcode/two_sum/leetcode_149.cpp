// Traverse all possiblities. 
class Solution {
public:
    int maxPoints(vector<vector<int>>& points) {
        if (points.size() < 3)
            return points.size();
        int i = 0, res = 0;
        for (; i < points.size() - 1; i++)
            if (points[i][0] != points[i+1][0] || points[i][1] != points[i+1][1])
                break;
        if (i == points.size() - 1)
            return points.size();
        
        for (i = 0; i < points.size() - 1; i++)
            for (int j = i + 1; j < points.size(); j++) {
                if (points[i][0] == points[j][0] && points[i][1] == points[j][1])
                    continue;
                int tempMax = 0;
                for (int k = 0; k < points.size(); k++) {
                    if (k == i || k == j)
                        continue;
                    if (check(points, i, j, k))
                        tempMax++;
                }
                res = tempMax > res ? tempMax : res;
            }
        return res + 2;
    }

    bool check(vector<vector<int>>& points, int i, int j, int k) {
        if (points[j][0] == points[k][0] && points[j][1] == points[k][1])
            return true;
        long long int g1 = gcd(points[j][1] - points[i][1], points[j][0] - points[i][0]),
                      g2 = gcd(points[k][1] - points[j][1], points[k][0] - points[j][0]);  // Will exceed the int range.
        
        return ((points[i][1] - points[j][1]) / g1 * ((points[k][0] - points[j][0]) / g2)) ==
               ((points[i][0] - points[j][0]) / g1 * ((points[k][1] - points[j][1]) / g2));
    }

    long long int gcd(int a, int b) {
        while (b != 0) {
            int temp = a % b;
            a = b;
            b = temp;
        }
        return a;
    }
};
