#include <stdio.h>
int main()
{  
  char input;
  int letter = 0,
      space = 0,
      number = 0,
      other = 0;
   
   printf("Please input the string end with Enter:\n");
   while ((input=getchar())!='\n')        // Special attention to understanding the loop condition - continuously accept input.
     {  if ((input>='a' && input<='z')||(input>='A' && input<='Z'))  // No need to find out ASCII.
          letter++;
        else if (input==' ')
          space++;
        else if (input>='0' && input <='9')
          number++;
        else
          other++;
	   }  
	printf("%d letter(s) inputed.\n"
	       "%d space(s) inputed.\n"
		   "%d number(s) inputed.\n"
		   "%d other char(s) inputed.\n",letter,space,number,other);  // Attention to the format used.
   
 } 
