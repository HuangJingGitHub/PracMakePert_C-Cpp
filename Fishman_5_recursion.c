#include <stdio.h>
int fish(int n, int x)
{  
  if((x-1)%5 == 0)
  {  if (n == 1)
       return 1;
     else
       return fish(n-1, (x-1)/5*4);
  }
  return 0;
 } 
 
 int main()
 {  
   int i=0, flag=0,x;
   do
   {
   	  x=i*5+1;
   	  i=i+1;
   	  if(fish(5,x))
   	  {  
   	    flag=1;
   	    printf("The fish caught is: %d\n",x);
	  }
   }
   while(!flag);
 }
