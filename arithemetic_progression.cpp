/* Function: 求等差数列第n项*/ 
#include <stdio.h>
int an(int a1, int dif, int n)
{ 
  int x;
  if (n==1)
  return a1;
  else 
     x = an(a1, dif, n-1) + dif; 
  return x;
}
int main()
{  int a,d,n;
   printf("Please input a1, d, n:\n");
   scanf("%d,%d,%d",&a, &d, &n);
   printf("a1 = %d\n",a);
   printf("d = %d\n",d);
   printf("n = %d\n",n);
   printf("a%d = %d\n",n, an(a,d,n));
}
