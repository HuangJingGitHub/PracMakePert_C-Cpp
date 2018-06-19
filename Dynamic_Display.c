#include <stdio.h>
#include <windows.h>

void SlowPrint(char *string, int sleeptime)
{  int i = 0;
   while(string[i] != '\0')
   {  printf("%c", string[i]);
      i++;
      Sleep(sleeptime);
   }
 } 
 
 int main()
 {  char filename[30], buffer[200];
    FILE *fp;
    
    printf("Please input the file name:\n");
    scanf("%s",filename);
    
    if((fp=fopen(filename, "rb")) == NULL)
    { perror(filename);
      return 0;
	}
	
	while(fgets(buffer,200,fp) != NULL)
	  SlowPrint(buffer,50);
	
 }
