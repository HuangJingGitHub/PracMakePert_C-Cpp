#include <stdio.h>

int calCow(int year)
{  int cowNum = 1, i;
   for(i=1; i<= year; i++)
     if(i>=4)
       if((year-i)>3)
          cowNum += calCow(year-i);
        else
          cowNum++;
   return cowNum;
}

int main()
{  int year=20;
   printf("%d\n", calCow(year));
} 
