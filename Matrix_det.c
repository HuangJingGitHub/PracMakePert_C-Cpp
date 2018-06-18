#include <stdio.h>
#include <stdlib.h>

int main()
{  float a[20][20], k, det=1.0;
   int n, r, c, i, j, flag = 0;
   
   printf("Please input the degree of the matrix:\n");
   scanf("%d",&n);
   printf("Please input the entries row by row:\n");
   for(i=0, j=0; i<n; i++, j=0)
   {  do
      {  scanf("%f",&a[i][j]);
         j++;
         if(j>n)
         {  printf("INPUT DIMENSION ERROR !");
            exit(1);
         }
      }
      while((getchar())!='\n');
   }
   
   for(r=0; r<=n-1; r++)
   {  if(a[r][r]==0)
      { for(i=r; i<=n-1; i++)
        {  if(a[i][r]!=0)
           {  flag = 1;
		      for(j=r; j<=n-1; j++)
              a[r][j] = a[r][j] + a[i][j];
              break;    
           }
	    }
	   if(flag==0)
	   {  printf("det=0\n");
	      exit(0);
	   }
      }
      
     for(i=r+1; i<=n-1; i++)
     { k = -a[i][r]/a[r][r];
	   for(j=r; j<=n-1; j++)
		     a[i][j] = a[i][j] + k*a[r][j];
     }
   det = det * a[r][r];
   }
   printf("The determinant is %.2f.\n", det);
}
