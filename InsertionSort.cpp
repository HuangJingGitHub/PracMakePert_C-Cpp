class InsertionSort{
public:
    void sort(vector<int>& unsorted){
      for (int i = 1; i < unsorted.size(); i++){
        int current = unsorted[i];
        int j = i-1;
        while(j > -1 && unsorted[j] > current){
          unsorted[j+1] = undorted[j];
          unsorted[j] = current;
        }
      }
};
