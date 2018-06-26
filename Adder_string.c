/*Function: 将字符串中数字前加上$*/ 
#include <stdio.h>

int Adder(char *s)
{
   int i, j;      // 字符串的复制比较思路 
   char t[100];
   for(i=0; s[i]!='\0'; i++)
     t[i]=s[i];
	 t[i]='\0'; 
   for(i=0, j=0; t[i]!='\0'; i++,j++)
   {  if(t[i]>='0' && t[i]<='9')
      {  s[j]='$';
         j++;
         s[j]=t[i];
	   }
	  else
	    s[j]=t[i];
   }
   s = '\0';   // 在末尾加一个字符串结尾标志，否则可能在末尾留有未知值
   return 0;
}

int main()
{  
   char ori[100];
   printf("Please input the string:\n");
   scanf("%s",ori);
   Adder(ori);
   printf("The new string is: %s\n",ori);
   
}
