struct DLinkedNode {
    int key, value;
    DLinkedNode* prev;
    DLinkedNode* next;
    DLinkedNode(): key(0), value(0), prev(nullptr), next(nullptr) {}
    DLinkedNode(int _key, int _value): key(_key), value(_value), prev(nullptr), next(nullptr) {}
};

class LRUCache {
private:
    unordered_map<int, DLinkedNode*> cache;
    DLinkedNode* head;
    DLinkedNode* tail;
    int size;
    int capacity_;

public:
    LRUCache(int capacity) capacity_(capacity) size(0) {
        head = new DLinkedNode();
        tail = new DLinkedNode();
        head->next = tail;
        tail->prev = head;
    }
    
    int get(int key) {
        if (cache.find(key) == cache.end())
            return -1;
        DLinkedNode* node = cache[key];
        moveToHead(node);
    }
    
    void put(int key, int value) {
        if (cache.size() == size && cache.find(key) == cache.end()) {
            cache.erase(useLog.front());
            useLog.pop();
        } 
        cache[key] = value;
        if (!useLog.empty() && useLog.front() == key)
            useLog.pop();
        if (useLog.empty() || useLog.back() != key)
            useLog.push(key);
    }
};

/**
 * Your LRUCache object will be instantiated and called as such:
 * LRUCache* obj = new LRUCache(capacity);
 * int param_1 = obj->get(key);
 * obj->put(key,value);
 */
