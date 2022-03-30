class Twitter {
public:
    int timeStamp_ = 0;
    const int newsFeedNum_ = 10;
    map<int, set<int>> userFollower_;
    map<int, set<int>> userFollowed_;
    map<int, vector<int>> userPostTimeLog_;
    map<int, priority_queue<int>> userNewsFeed_;
    map<int, int> timeToPost_;

    Twitter() {

    }
    
    void postTweet(int userId, int tweetId) {
        timeToPost_[timeStamp_] = tweetId;
        userPostTimeLog_[userId].push_back(timeStamp_);
        userFollower_[userId].insert(userId);   // ensure the account follows itself
        userFollowed_[userId].insert(userId);

        for (auto it = userFollower_[userId].begin(); it != userFollower_[userId].end(); it++) {
            int curFollowerId = *it;
            if (userNewsFeed_[curFollowerId].size() < newsFeedNum_)
                userNewsFeed_[curFollowerId].push(timeStamp_);
            else {
                vector<int> tempStore(newsFeedNum_ - 1);
                for (int i = 0; i < newsFeedNum_ - 1; i++) {
                    tempStore[i] = userNewsFeed_[curFollowerId].top();
                    userNewsFeed_[curFollowerId].pop();
                }
                userNewsFeed_[curFollowerId].pop();

                userNewsFeed_[curFollowerId].push(timeStamp_);
                for (int time : tempStore)
                    userNewsFeed_[curFollowerId].push(time);
            }
        }
        timeStamp_++;
    }
    
    vector<int> getNewsFeed(int userId) {
        vector<int> res;
        auto newsFeed = userNewsFeed_[userId];

        while (newsFeed.size() != 0) {
            res.push_back(timeToPost_[newsFeed.top()]);
            newsFeed.pop();
        }
        return res;
    }
    
    void follow(int followerId, int followeeId) {
        if (userFollowed_[followerId].find(followeeId) != userFollowed_[followerId].end())
            return;

        userFollower_[followeeId].insert(followerId);
        userFollowed_[followerId].insert(followeeId);
        followRefreshNewsFeed(followerId, followeeId);
    }
    
    void unfollow(int followerId, int followeeId) {
        if (userFollowed_[followerId].find(followeeId) == userFollowed_[followerId].end())
            return;

        userFollower_[followeeId].erase(followerId);
        userFollowed_[followerId].erase(followeeId);
        unfollowRefreshNewsFeed(followerId, followeeId);
    }

    void followRefreshNewsFeed(int followerId, int followeeId) {
        vector<int> tempStore;

        while (userNewsFeed_[followerId].size() != 0) {
            tempStore.push_back(userNewsFeed_[followerId].top());
            userNewsFeed_[followerId].pop();
        }

        for (int followeeNewsTime : userPostTimeLog_[followeeId]) 
                tempStore.push_back(followeeNewsTime);
        
        sort(tempStore.begin(), tempStore.end(), greater<int>());

        int idx = 0;
        while (userNewsFeed_[followerId].size() < newsFeedNum_ && idx < tempStore.size())
            userNewsFeed_[followerId].push(tempStore[idx++]);
    }

    void unfollowRefreshNewsFeed(int followerId, int followeeId) {
        vector<int> tempStore;
        
        while (userNewsFeed_[followerId].size() != 0)
            userNewsFeed_[followerId].pop();

        for (auto it = userFollowed_[followerId].begin(); it != userFollowed_[followerId].end(); it++) {
            int curFollowee = *it;
            for (int& time : userPostTimeLog_[curFollowee])
                tempStore.push_back(time);
        }
        sort(tempStore.begin(), tempStore.end(), greater<int>());

        int idx = 0;
        while (userNewsFeed_[followerId].size() < newsFeedNum_ && idx < tempStore.size())
            userNewsFeed_[followerId].push(tempStore[idx++]);
    }
};

/**
 * Your Twitter object will be instantiated and called as such:
 * Twitter* obj = new Twitter();
 * obj->postTweet(userId,tweetId);
 * vector<int> param_2 = obj->getNewsFeed(userId);
 * obj->follow(followerId,followeeId);
 * obj->unfollow(followerId,followeeId);
 */
