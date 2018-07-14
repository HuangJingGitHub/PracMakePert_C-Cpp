#include <stdio.h>
int main()
{ double number1 =0.0, number2 = 0.0;
  char operation = 0;
  
  printf("Start:\n");
  scanf("%lf%c%lf", &number1, &operation, &number2);
  
  switch(operation)
  {  case '+':
  	   printf("=%.2lf", number1 + number2);
  	   break;
  	 case '-':
  	   printf("=%.2lf", number1 - number2);
  	   break;
  	 case '*':
  	   printf("=%.2lf", number1 * number2);
  	   break;
  	 case '/':
  	   if(number2 == 0)
  	     printf("ERROR: Divisor is 0!");
  	     printf("=%.2lf", number1 / number2);  
  	     break;
  	 case '%':   
  	   if((int)number2 == 0)
		 printf("ERROR: Divisor is 0!");
	     printf("%.2lf", (int)number1 / (int)number2);   // 余数定义再整数除法中，对于允许小数的运算没有定义
	     break;
	 default:
	 	printf("ERROR: Invalid operation!");
   } 
   return 0;
  } 
