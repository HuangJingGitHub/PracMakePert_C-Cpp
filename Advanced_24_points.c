/* 程序基于24_points和permutation两个小程序，这两个单独的程序功能都比较局限，
24_points只能给出特定顺序(即输入顺序)的结果，permutation可以给出n的全排列，所以将两者
结合可以给出24点问题的所有可能解法。 */
 
#include <stdio.h>
int flag = 0;

float cal(float x, int op, float y)
{  
   switch(op)
   {  case 1: return x+y;
      case 2: return x-y;
      case 3: return x*y;
      case 4: return x/y;
      default: return 0.0; 
   }
}

char op(int i)
{ 
   switch(i)
   {  case 1: return '+';
      case 2: return '-';
      case 3: return '*';
      case 4: return '/';
      default: return ' ';
   } 
}

void twenty_four(float *poker)
{ 
   int i, op1,op2,op3,op4;
   float A, B, C, D, ans;
   A = poker[0];
   B = poker[1];
   C = poker[2];
   D = poker[3];
    
    for(op1=1; op1<5; op1++)
      for(op2=1; op2<5; op2++)
        for(op3=1; op3<5; op3++)
        {  
		   ans = cal(cal(A,op1,B), op2, cal(C,op3,D));
           if(ans==24)
           {
              printf("(%-2.0f %c %2.0f) %c (%-2.0f %c %-2.0f) = 24\n",
			         A,op(op1),B,op(op2),C,op(op3),D);
              flag = 1;
           }
           
           ans = cal(cal(cal(A, op1, B), op2, C), op3, D);
           if(ans==24)
           {
              printf("((%-2.0f %c %-2.0f) %c %-2.0f) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
              flag = 1;
           }
           
		   ans =  cal(cal(A, op1, cal(B, op2, C)), op3, D);
           if(ans==24)
           {
              printf("(%-2.0f %c (%-2.0f  %c %-2.0f)) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
              flag = 1;
           }
           
		   ans = cal(A, op1, cal(cal(B, op2, C), op3, D));
		   if(ans==24)
		   {
              printf("(%-2.0f %c ((%-2.0f  %c %-2.0f) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
			  flag = 1;
		   }
		   
		   ans = cal(A, op1, cal(B, op2, cal(C, op3, D)));	
		   if(ans==24)
		   {
              printf("(%-2.0f %c (%-2.0f  %c (%-2.0f %c %-2.0f)) = 24\n",
			         A,op(op1),B,op(op2),C,op(op3),D);	
			  flag = 1;
		    }
		}
} 

void swap(float *p1, float *p2)
{  
   float t = *p1;
   *p1 = *p2;
   *p2 = t;
}

void permutation(float a[], int index, int size)
{  
   int i, j;
   if(index == size)
   {  
          twenty_four(a);
          printf("\n");   
   }
   else
   {  for(j=index; j<size; j++)
      {  swap(&a[j],&a[index]);
         permutation(a, index+1, size);
         swap(&a[j],&a[index]);
      } 
   }
}

int main() 
{   
    int i;
    float a[4];
    printf("Please input the 4 numbers(1-13):\n");
    for(i=0; i<4; i++)
    {  scanf("%f",&a[i]);
       if(!(a[i]>=1 && a[i]<=13))
       {  printf("Invalid number! Please input again:\n");
          scanf("%f",&a[i]);
       }
    }
    printf("The possible calculations to generate 24 are:\n"); 
    permutation(a, 0, 4);
    if(flag == 0)
    printf("The 4 given numbers cannot generate 24. \n");   
}
