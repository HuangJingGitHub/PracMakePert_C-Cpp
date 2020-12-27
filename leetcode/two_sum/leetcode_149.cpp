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
               ((points[i][0] - points[j][0]) / g1 * ((points[k][1] - points[j][1]) / g2));  // Detail: Using multiplication instead of (y2 - y1) / (x2 - x1) = (y - y2) / (x - x2)
    }                                                                                        // is more general as it can include vertial line case.                              

    long long int gcd(int a, int b) {
        while (b != 0) {
            int temp = a % b;
            a = b;
            b = temp;
        }
        return a;
    }
};


// Store line info to decrease unnecessary reptations.
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
        
        unordered_set<string> lineCoef; // y = kx + b -> k, b
        for (i = 0; i < points.size() - 1; i++)
            for (int j = i + 1; j < points.size(); j++) {
                if (points[j][0] == points[i][0] && points[j][1] == points[i][1])
                    continue;
                string curLineCoef = getLineCoef(points, i, j);
                if (lineCoef.find(curLineCoef) != lineCoef.end())
                    continue;
                
                lineCoef.insert(curLineCoef);
                int tempMax = 0;
                for (int k = 0; k < points.size(); k++) {
                    if (k != i && k != j)
                        if (check(points, i, j, k))
                            tempMax++;
                }
                res = tempMax > res ? tempMax : res;
            }
        return res + 2;
    }

    string getLineCoef(vector<vector<int>>& points, int i, int j) {
        string res;
        // x1 == x2
        if (points[i][0] == points[j][0]) {
            res = "Line_X=" + to_string(points[i][0]);
            return res;
        }
        long int x1 = points[i][0], x2 = points[j][0],
                y1 = points[i][1], y2 = points[j][1];
        int g1 = gcd(y2 - y1, x2 - x1), g2 = gcd(x2 * y1 - x1 * y2, x2 - x1);
        string k = to_string((y2 - y1) / g1) + "/" + to_string((x2 - x1) / g1),
               b = to_string((x2 * y1 - x1 * y2) / g2) + "/" + to_string((x2 - x1) / g2);
        res = k + "@" + b;
        return res;
    }     

    int gcd(int a, int b) {
        while (b != 0) {
            int temp = a % b;
            a = b;
            b = temp;
        }
        return a;
    }

    bool check(vector<vector<int>>& points, int i, int j, int k) {
        long long t1 = (points[j][1] - points[i][1]);
        t1 *= (points[k][0] - points[j][0]);
        long long  t2 = (points[k][1] - points[j][1]);
        t2 *= (points[j][0] - points[i][0]);
        return t1 == t2;
    }
};
