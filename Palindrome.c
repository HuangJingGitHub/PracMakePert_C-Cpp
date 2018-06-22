#include <stdio.h>   // 回文字符判断 
#include <string.h>

void main()
{  char *ch, s1, s2;
   int len, i, flag = 0, state;
   
   do
   { flag = 0;                    // 记得每次循环都要初始化标志 
     printf("Please input the string:\n");
     scanf("%s",ch);
   
     len = strlen(ch);
     for(i=0; ch[i]==ch[len-1-i]; i++)
     {  if(i==len-1-i || i+1==len-1-i)
          flag = 1;
     }
     if(flag)
       printf("The string is a palindrome.\n");
     else
       printf("The string is not a palindrome.\n");
    do
    {
      printf("Continue ?--->1 Yes  --->0 NO\n");
      scanf("%d", &state);
      if(!(state == 0 || state == 1))
      printf("Invalid input!\n");
    }
    while(!(state == 0 || state == 1));   // do while结构记得加while后的分号 
   }
   while(state); 
 } 
