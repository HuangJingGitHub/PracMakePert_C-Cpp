#include <stdio.h>    // catch 30的游戏规则：两方轮流计数，首发随机，第一次可选1或2，之后计数可以加1或加2，但不允许出现3个数连续的情况， 
#include <stdlib.h>   // 先达到30者获胜。 由倒推可知，获胜策略是把握住3的倍数 
#include <time.h>
#include <math.h>

int input(int cs);  // cs:current sum
int computer_put(int cs);
int judge_series(int *a, int t1, int t2);

int dif[2] = {0, 0};  // 用来记录出数差的全局变量 

int main()
{  int sum = 0;
   srand((unsigned)time(NULL));  // 注意C中实现随机数输出的配置是：stdlib.h time.h srand((unsigned)time(NULL)) rand() 仅rand()每次输出不变 
   if(rand()%2)                  // 产生随机数0或1，1则玩家先出数 
   {  sum = input(sum);
      dif[0] = sum - 0;          // 初始化记录出数差的全局变量 
	} 
     
   while(sum!=30)               // 注意与前面语句之间的逻辑，无论谁先发，都可以实现轮流出数 
     if((sum=computer_put(sum))==30)
       printf("*****Sorry You Lose The Game!*****\n");
     else if((sum=input(sum))==30)
       printf("*****Congratulations！ You Win The Game!*****");
     
 }

int judge_series(int *a, int t1, int t2)
{  *a = *(a+1);
   *(a+1) = t2 - t1;
   printf("***dif1 = %d dif2=%d***\n",*a,*(a+1));
   if(*a==1 && *(a+1)==1)
     return 1;
   else
     return 0;
}

int input(int cs)
{  int temp, flag;
   printf("Please input the number:\n");
   scanf("%d",&temp);
   flag = judge_series(dif, cs, temp);
   printf("***flag = %d***\n",flag);
   while((temp-cs)>2 || (temp-cs)<1 || flag || temp>30 )
   {  printf("Invalid input! Please input again:\n");
      scanf("%d",&temp);
      flag = judge_series(dif, cs, temp);    // 判断同时在judge_series()中会记录更新出数差 
   }
   printf("You Count: %d \n", temp);
   return temp;
 } 
 
int computer_put(int cs)
{  int temp;
   if(cs%3==1)
   {  cs+=2;
      dif[0] = dif[1];                      // 更新出数差 
      dif[1] = 2; 
      printf("Computer Counts: %d \n", cs);
   }
   else if(cs%3==2)
   {  cs = cs + 1;
      dif[0] = dif[1];
      dif[1] = 1; 
      printf("Computer Counts: %d \n", cs);
   }
   else
   {  temp = rand()%2 + 1;
      cs = cs + temp;
      dif[0] = dif[1];
      dif[1] = temp; 
      printf("Computer Counts: %d \n", cs);
   }

   return cs;
     
}

