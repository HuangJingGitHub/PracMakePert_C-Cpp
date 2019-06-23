#include <stdio.h>

float cal(float x, int op, float y)
{  switch(op)
   {  case 1: return x+y;
      case 2: return x-y;
      case 3: return x*y;
      case 4: return x/y;
      default: return 0.0; 
   }
}

char op(int i)
{  switch(i)
   {  case 1: return '+';
      case 2: return '-';
      case 3: return '*';
      case 4: return '/';
      default: return ' ';
   } 
}

// 实现扑克牌中24点的程序，主要的注意点：(1) A,B,C,D四个数中间的三个运算
// 遍历和五种括号顺序的遍历，函数是强大的工具：(2) 输出的不同的运算符号也是
// 通过调用定义的函数实现的。 
int main()
{  int i,flag = 0, flag1, op1,op2,op3;
   float a[4], A, B, C, D, ans;
    
    printf("Please input the 4 numbers(1-13):\n");
    for(i=0; i<4; i++)
    {  scanf("%f",&a[i]);
       flag1 = 1;	
      if(!(a[i] >= 1 && a[i] <= 13))
      {  while(flag1)
         {  printf("Invalid number! Please input again:\n");
            scanf("%f",&a[i]);
	    if(a[i] >= 1 && a[i] <= 13)
              flag1 = 0;
	 }
      }
    }
    A = a[0]; B = a[1]; C = a[2]; D = a[3];
    
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
           if(ans == 24)
           {
              printf("((%-2.0f %c %-2.0f) %c %-2.0f) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
              flag = 1;
           }
           
		   ans =  cal(cal(A, op1, cal(B, op2, C)), op3, D);
           if(ans == 24)
           {
              printf("(%-2.0f %c (%-2.0f  %c %-2.0f)) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
              flag = 1;
           }
           
		   ans = cal(A, op1, cal(cal(B, op2, C), op3, D));
		   if(ans == 24)
		   {
              printf("(%-2.0f %c ((%-2.0f  %c %-2.0f) %c %-2.0f) = 24\n",
			          A,op(op1),B,op(op2),C,op(op3),D);
			  flag = 1;
		   }
		   
		   ans = cal(A, op1, cal(B, op2, cal(C, op3, D)));	
		   if(ans == 24)
		   {
              printf("(%-2.0f %c (%-2.0f  %c (%-2.0f %c %-2.0f)) = 24\n",
			         A,op(op1),B,op(op2),C,op(op3),D);	
			  flag = 1;
		    }
		}
	if(flag == 0)
	printf("The 4 given numbers cannot produce 24. \n");
    
} 
