#include <stdio.h>
#include <stdlib.h>

float det(float a[20][20], int n)
{  float k, det=1.0;
   int r, i, j, flag = 0;
   
   for(r=0; r<=n-1; r++)
   {  if(a[r][r]==0)
      { for(i=r; i<=n-1; i++)
        {  if(a[i][r]!=0)               // 如果有主对角线上的元素为零的情况需要进行处理，向下 
           {  flag = 1;                 // 检索该列元素不为零的行，加至为零行，不改变行列式的值 
		      for(j=r; j<=n-1; j++)
              a[r][j] = a[r][j] + a[i][j];
              break;    
           }
	    }
	   if(flag==0)
	   {  det = 0;
//	      printf("det=0\n");
	      return det;
	   }
      }
      
     for(i=r+1; i<=n-1; i++)                // 对角化过程注意细节，例如一定要用 k 把系数存下，否则之后用的 
     { k = -a[i][r]/a[r][r];                // 系数将是0 
	   for(j=r; j<=n-1; j++)
		     a[i][j] = a[i][j] + k*a[r][j];
     }
   det = det * a[r][r];
   }
   return det;
//   printf("The determinant is %.2f.\n", det);
}

int main()
{
  float a[20][20], temp[20][20] = {0}, inv[20][20] = {0}, det_a;
   int n, i, j, i1, j1, i2, j2, flag1 = 0, flag2 = 0;
   
   printf("Please input the degree of the matrix:\n");
   scanf("%d",&n);
   printf("Please input the entries row by row:\n");
   
   for(i=0, j=0; i<n; i++, j=0)        // 注意输入矩阵的方法 
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
   det_a = det(a,n);
   
   for(i = 0; i<n; i++)
     for(j = 0; j<n; j++)
     {  for(i1 = 0; i1<n; i1++)
           if(i1==i)
           { flag1 = 1;
               continue;
		   }
           else
             for(j1 = 0; j1<n; j1++)
             {
			    if(j1==j)
                { flag2 = 1;
                   continue;
				}
				if((!flag1) && (!flag2))
				  temp[i1][j1] = a[i1][j1];
				else if((flag1) && (!flag2))
				  temp[i1-1][j1] = a[i1][j1];
				else if((!flag1) && (flag2))
				  temp[i1][j1-1] = a[i1][j1];
				else
				  temp[i1-1][j1-1] = a[i1][j1];
		//		printf(">>>%.2f<<<\n", temp[i1][j1]);
			 } 
        
		inv[j][i] = det(temp, n-1) / det_a;     
    }
    
  printf(">>>%d<<<\n", n);
	    for(i2=0; i2<n-1; i2++)
	      for(j2=0; j2<n-1; j2++)
	       {  if(j2 == n-2)
	            printf("%.2f\n", temp[i2][j2]);
	          else
	            printf("%.2f  ", temp[i2][j2]);
	        }
	        
  
  for(i=0; i<n; i++)
    for(j=0; j<n; j++)
      if(j == n-1)
      printf("%.2f\n", inv[i][j]);
      else
      printf("%.2f  ", inv[i][j]);
}
 
