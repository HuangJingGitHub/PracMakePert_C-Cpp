#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float det(float A[20][20], int n)       // 求行列式的函数源于Matrix_det.c 
{  float k, det=1.0;
   int r, i, j, flag = 0;
   
   for(r=0; r<=n-1; r++)
   {  if(A[r][r]==0)
      { for(i=r; i<=n-1; i++)
        {  if(A[i][r]!=0)               // 如果有主对角线上的元素为零的情况需要进行处理，向下 
           {  flag = 1;                 // 检索该列元素不为零的行，加至为零行，不改变行列式的值 
	      for(j=r; j<=n-1; j++)
                A[r][j] = A[r][j] + A[i][j];
              break;    
           }
	    }
	   if(flag==0)
	   {  det = 0;
	      return det;
	   }
      }
      
     for(i=r+1; i<=n-1; i++)                // 对角化过程注意细节，例如一定要用 k 把系数存下，否则之后用的 
     { k = -A[i][r] / A[r][r];                // 系数将是0 
       for(j=r; j<=n-1; j++)
         A[i][j] = A[i][j] + k*A[r][j];
     }
   det = det * A[r][r];
   }
   return det;
}

int main()
{
  float a[20][20], 
        b[20][20], 
        temp[20][20], 
	inv[20][20] = {0}, 
	det_a;
   int n, i, j, i1, j1, i2, j2, 
       flag1 = 0, 
       flag2 = 0;
   
   printf("Please input the degree of the matrix:\n");
   scanf("%d",&n);
   printf("Please input the entries row by row:\n");
   
   for(i=0, j=0; i<n; i++, j=0)        // 注意输入矩阵的方法 
   {  do
      {  scanf("%f",&a[i][j]);         // 在输入矩阵a时，同时复制a到b，因为后面求行列式时会将矩阵对角化并替换原矩阵 
         b[i][j] = a[i][j];            // 但求余子式时需要用到原矩阵，所以复制a(以空间换时间）;另外也可以在求出伴随矩阵 
         j++;                          // 之后再计算a的行列式，但此时如果det(a) == 0，则伴随矩阵并没有用，浪费计算时间 
         if(j>n)
         {  printf("INPUT DIMENSION ERROR !");
            exit(1);
         }
      }
      while((getchar())!='\n');
   }
   
   det_a = det(b,n);      
   if(det_a == 0)
   {  printf("The matrix is singular!");
      exit(0);
   }
   
   for(i = 0; i<n; i++)
     for(j = 0; j<n; j++, flag1 = 0)          // 计数器，flag之类一定要注意是否需要循环初始化,有时不易察觉 
     {  for(i1 = 0; i1<n; i1++, flag2 = 0)
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
				                       
		 if((!flag1) && (!flag2))      // 余子式和原矩阵之间的脚标对应法则 
		   temp[i1][j1] = a[i1][j1];
		 else if((flag1) && (!flag2))
		   temp[i1-1][j1] = a[i1][j1];
		 else if((!flag1) && (flag2))
		   temp[i1][j1-1] = a[i1][j1];
		 else
		   temp[i1-1][j1-1] = a[i1][j1];
				  
	     }    
	inv[j][i] = pow(-1,i+j) * det(temp, n-1);  
    }
    
  printf("The inverse is:\n");
  for(i=0; i<n; i++)
    for(j=0; j<n; j++)
      if(j == n-1)
      printf("%-5.3f\n", inv[i][j] / det_a);
      else
      printf("%-5.3f  ", inv[i][j] / det_a);
}
 
 
