class Solution {
public:
    bool validSquare(vector<int>& p1, vector<int>& p2, vector<int>& p3, vector<int>& p4) {
        int sqrt_dis_1_2 = computeSqrDistance(p1, p2),
            sqrt_dis_1_3 = computeSqrDistance(p1, p3);
    
        // Assume 2, 3 are neighbors of 1. 1 and 4 are on the diagonal, i.e., -1-2-4-3-1-
        vector<int> side_1_2, side_1_3, side_2_4, side_3_4;
        if (sqrt_dis_1_2 < sqrt_dis_1_3) {    
            auto temp = p3;
            p3 = p4;
            p4 = temp;        
        }
        else if (sqrt_dis_1_2 > sqrt_dis_1_3) {
            auto temp = p2;
            p2 = p4;
            p4 = temp;
        }
        side_1_2 = {p2[0] - p1[0], p2[1] - p1[1]};
        side_1_3 = {p3[0] - p1[0], p3[1] - p1[1]};
        side_2_4 = {p4[0] - p2[0], p4[1] - p2[1]};
        side_3_4 = {p4[0] - p3[0], p4[1] - p3[1]}; 

        int len_1_2 = computeSqrDistance(p1, p2),
            len_1_3 = computeSqrDistance(p1, p3),
            len_2_4 = computeSqrDistance(p2, p4),
            len_3_4 = computeSqrDistance(p3, p4),
            inner_product_2_1_3 = computeInnerProduct(side_1_2, side_1_3),
            inner_product_1_2_4 = computeInnerProduct(side_1_2, side_2_4),
            inner_product_1_3_4 = computeInnerProduct(side_1_3, side_3_4);

        return len_1_2 > 0 && len_1_2 == len_1_3 && len_1_2 == len_2_4 && len_2_4 == len_3_4 &&
               inner_product_2_1_3 == 0 && inner_product_1_2_4 == 0 && inner_product_1_3_4 == 0;
    }

    int computeSqrDistance(vector<int>& p1, vector<int>& p2) {
        if (p1.size() != 2 || p2.size() != 2)
            return 0;
        return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]);
    }

    int computeInnerProduct(vector<int>& v1, vector<int>& v2) {
        if (v1.size() != 2 || v2.size() != 2)
            return 0;
        return v1[0] * v2[0] + v1[1] * v2[1];
    }
};


// More compact. Only two side lengths are possible for segments linking any two vertices of a square.
class Solution {
public:
    bool validSquare(vector<int>& p1, vector<int>& p2, vector<int>& p3, vector<int>& p4) {
        set<int> sqr_dist_set;

        sqr_dist_set.insert(computeSqrDistance(p1, p2));
        sqr_dist_set.insert(computeSqrDistance(p1, p3));
        sqr_dist_set.insert(computeSqrDistance(p1, p4));
        sqr_dist_set.insert(computeSqrDistance(p2, p3));
        sqr_dist_set.insert(computeSqrDistance(p2, p4));
        sqr_dist_set.insert(computeSqrDistance(p3, p4));

        return sqr_dist_set.size() == 2 && sqr_dist_set.count(0) == 0;
    }

    int computeSqrDistance(vector<int>& p1, vector<int>& p2) {
        if (p1.size() != 2 || p2.size() != 2)
            return 0;
        return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]);
    }
};
