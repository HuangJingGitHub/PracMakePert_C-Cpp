#include <stdio.h>
int main()
{ int n1,n2,n3;
  for (n1=0; n1<=100/5; n1++)
    {  for (n2=0; n2*2<=100-n1*5; n2++)
         { n3 = 100 - n1 - n2;
           if (n3%3==0 && 5*n1 + 3*n2 + n3/3==100)
           printf("There are %-2d rooster(s), %-2d hen(s), %-2d child chicken(s)\n",n1,n2,n3);
		 }
	}
}
