class MyCalendar {
public:
    set<int> bookStart;
    set<int> bookEnd;
    MyCalendar() { }
    
    bool book(int start, int end) {
        if (bookStart.empty()){
            bookStart.insert(start);
            bookEnd.insert(end);
            return true;
        }

        int preStart = binarySearch(start);
        if (preStart == -2)
            return false;
        else if (preStart == -1){
            if (end <= *(bookStart.begin())){
                bookStart.insert(start);
                bookEnd.insert(end);
                return true;
            }
            else
                return false;
        }
        else if (preStart == bookStart.size() - 1){
            if (start >= *bookEnd.rbegin()){
                bookStart.insert(start);
                bookEnd.insert(end);
                return true;
            }
            else    
                return false;
        }
        else{
            auto it1 = bookStart.begin(), it2 = bookEnd.begin();
            for (int i = 0; i < preStart; i++){
                it1++;
                it2++;
            }         
            it1++;   
            if (start >= *it2 && end <= *it1){
                bookStart.insert(start);
                bookEnd.insert(end);
                return true;
            }
            else
                return false;
        }
    }

    int binarySearch(int start){
        set<int>::iterator left = bookStart.begin(), right = bookStart.begin();
        for (int i = 0; i < bookStart.size() -1; i++)
            right++;
        for ( ;right - left >= 0;){
            set<int>::iterator mid = left + (right - left) / 2;
            if (*mid == start)
                return -2;
            
            if (*mid < start)
                left = mid + 1;
            else
                right = mid - 1;
        }
        return right - bookStart.begin();
    }
};

/**
 * Your MyCalendar object will be instantiated and called as such:
 * MyCalendar* obj = new MyCalendar();
 * bool param_1 = obj->book(start,end);
 */
