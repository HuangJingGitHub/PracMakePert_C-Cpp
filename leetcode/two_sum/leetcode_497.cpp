class Solution {
public:
    vector<vector<int>> rects_;
    vector<int> accumulated_area_;
    int rect_num_;
    int total_area_;
    Solution(vector<vector<int>>& rects) {
        srand(time(NULL));
        rects_ = rects;
        rect_num_ = rects_.size();
        accumulated_area_ = vector<int>(rect_num_, 0);
        
        accumulated_area_[0] = (rects_[0][2] - rects_[0][0] + 1) * (rects_[0][3] - rects_[0][1] + 1);
        for (int i = 1; i < rect_num_; i++)
            accumulated_area_[i] = accumulated_area_[i - 1] + (rects_[i][2] - rects_[i][0] + 1) * (rects_[i][3] - rects_[i][1] + 1);
            // Note that side lengths need to be added by 1 since we are sampling integer points covered by the rectangle uniformly, not 
            // the area covered by the rectangle.
        total_area_ = accumulated_area_.back();
    }
    
    vector<int> pick() {
        int rand_area = rand() % total_area_ + 1;
        int rect_idx = findRectByArea(rand_area);
        int width = rects_[rect_idx][2] - rects_[rect_idx][0],
            height = rects_[rect_idx][3] - rects_[rect_idx][1],
            res_x = rects_[rect_idx][0] + rand() % (width + 1),
            res_y = rects_[rect_idx][1] + rand() % (height + 1);
        return {res_x, res_y};
    }

    int findRectByArea(int rand_area) {
        if (rand_area <= 0 || rand_area > total_area_)
            return -1;

        int left = 0, right = rect_num_ - 1, mid = 0;
        while (left < right) {
            mid = left + (right - left) / 2;
            if ((mid > 0 && accumulated_area_[mid] >= rand_area && rand_area > accumulated_area_[mid - 1])
                || (mid == 0 && rand_area <= accumulated_area_[0]))
                return mid;
            else if (accumulated_area_[mid] < rand_area)
                left = mid + 1;
            else
                right = mid -1;
        }
        return left;
    }
};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(rects);
 * vector<int> param_1 = obj->pick();
 */
