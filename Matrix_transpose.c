#include <stdio.h>

int main()
{  float A[3][3]={1,2,3,4,5,6,7,8,9}, temp;
   int i, j;
   
   for(i=0; i<3; i++)
   {  for(j=0; j<3; j++)
      {  printf("Please input the matrix element a%d%d:\n",i+1,j+1);
         scanf("%f",&A[i][j]);
	  }
   }
   
   for(i=0; i<3; i++)
   {  for(j=i+1; j<3; j++)
      {  temp=A[i][j];
         A[i][j]=A[j][i];
         A[j][i]=temp;
	  }
   }
   
   printf("The transpose is:");
   for(i=0; i<3; i++)
   {  printf("\n"); 
      for(j=0; j<3; j++)
        printf("%-10.2f",A[i][j]);	  
   }
   
   
 } 
