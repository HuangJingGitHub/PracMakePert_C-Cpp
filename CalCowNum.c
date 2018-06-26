// Tencent的一个题目：开始一头奶牛，至第四年开始产崽，之后每年产一崽，计算n年后一共有多少奶牛 
#include <stdio.h>

int calCow(int year)
{  int cowNum = 1, i;
   for(i=1; i<= year; i++)
   {  if(i<4)
        cowNum = 1;
      else
        cowNum = cowNum + calCow(year-i);    // 使用递归解决，从年份的角度 
   }
   return cowNum;
}

int main()
{  int year, flag = 0;
   
   printf("Input the year:\n");
   while(flag == 0)
   {  scanf("%d",&year);
      if(year>=1)
   	flag = 1;
      else
      	printf("Invalid input. Please input the year:\n");
   }

   printf("Number of cow(s): %d\n", calCow(year));
}
