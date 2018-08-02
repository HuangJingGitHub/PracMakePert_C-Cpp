#include <stdio.h>

int main()
{  int a[20], i = 0, j, k, temp;
   
   printf("Please input the array:\n");
   do
   { scanf("%d", &a[i]);
     i++;
   }while((getchar()) != '\n');
   for(j = 0; j < i; j++)
     printf("%d  ",a[j]);
   
   for(k = 1; k < i; k++)
    for(j = 0; j < i-k; j++)
    { if(a[j] > a[j+1])
      { temp = a[j];
        a[j] = a[j+1];
        a[j+1] = temp;
	  }
    }
   
   printf("\n");
   for(j = 0; j < i; j++)
   printf("%d  ",a[j]);
 } 
