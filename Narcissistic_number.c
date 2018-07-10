#include <stdio.h>
#include <math.h>

int main()
{  int n = 100, unit, tens, hundreds, cubic = 500;
   for(; n<1000; n++)
   {  hundreds = n / 100;
      tens = (n - hundreds*100) / 10;
      unit = n % 10;
      
      cubic = pow(unit, 3) + pow(tens, 3) + pow(hundreds, 3);
      if( cubic == n)
	  printf("Narcissistic Number: %d \n", n); 
   }
   return 0;
}
