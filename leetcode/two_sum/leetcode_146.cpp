// good example of application of double linked list 
// implementation of LRU cache mechanism  
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
    int cacheCapacity;

public:
    LRUCache(int capacity): cacheCapacity(capacity), size(0) {
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
        return node->value;
    }
    
    void put(int key, int value) {
        if (cache.find(key) == cache.end()) {
            DLinkedNode* node = new DLinkedNode(key, value);
            cache[key] = node;
            addToHead(node);
            size++;
            if (size > cacheCapacity) {
                DLinkedNode* removed = removeTail();
                cache.erase(removed->key);
                delete removed;
                size--;
            }
        }
        else {
            DLinkedNode* node = cache[key];
            node->value = value;
            moveToHead(node);
        }
    }

    void addToHead(DLinkedNode* node) {
        node->prev = head;
        node->next = head->next;
        head->next = node;
        node->next->prev = node;
    }
    
    void moveToHead(DLinkedNode* node) {
        node->prev->next = node->next;
        node->next->prev = node->prev;
        addToHead(node);
    }

    DLinkedNode* removeTail() {
        DLinkedNode* tailNode = tail->prev;
        tailNode->prev->next = tail;
        tail->prev = tailNode->prev;
        return tailNode;
    }
};

/**
 * Your LRUCache object will be instantiated and called as such:
 * LRUCache* obj = new LRUCache(capacity);
 * int param_1 = obj->get(key);
 * obj->put(key,value);
 */
