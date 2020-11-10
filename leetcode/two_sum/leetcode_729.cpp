// set keeps key order
// use of lower_bound() vs manual binary search
class MyCalendar {
public:
    set<pair<int, int>> bookedLog;

    // By inserting boundarying pairs, newly inserted {start, end}
    // does not need to be dealt with specially if it appears beyond
    // booked pair boundaries.
    MyCalendar() {
        bookedLog.insert({INT_MIN, INT_MIN});
        bookedLog.insert({INT_MAX, INT_MAX});
    }
    
    bool book(int start, int end) {
        set<pair<int, int>>::iterator nextPos = bookedLog.lower_bound({start, end});
        if (nextPos->first < end)
            return false;
        if ((--nextPos)->second > start)
            return false;
        bookedLog.insert({start, end});
        return true;
    }
};
