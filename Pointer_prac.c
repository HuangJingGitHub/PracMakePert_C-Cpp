#include <stdio.h>
#include <stdlib.h>

int main()http://c.biancheng.net/cpp/html/2825.html
{  char *lines[5] = {                          // 小细节，字符串之间的逗号不要少，不然可以通过编译，但运行会崩溃 
                    "Controllability",         // 指向字符的指针数组， 理解可以看看解析 http://c.biancheng.net/cpp/html/2825.html
                    "means that for any initial state x(0)=x0",
                    "and any final state x1, ",
                    "there exists an input that transfers x0 to x1",
                    "in a finite time."
		          };
	
	char *str1 = lines[1],
	     *str2 = *(lines + 3),
	     c1 = *(*(lines + 4) + 6),
	     c2 = (*lines + 5)[5],
	     c3 = *lines[0] + 2;
    
	printf("ste1 = %s\n", str1);
	printf("ste2 = %s\n", str2);
	printf("  c1 = %c\n", c1);
	printf("  c2 = %c\n", c2);
	printf("  c3 = %c\n", c3);
	
	return EXIT_SUCCESS;     // stdlib.h中的宏定义 EXIT_SCUESS 0, EXIT_FAILURE 1 
 } 
