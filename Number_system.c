#include <stdio.h>
#define Maxchar 101

int char2num(char ch);
char num2char(int num);
long source2decimal(char temp[], int source);
int decimal2obj(char temp[], long decimal_num, int obj);
void output(char temp[], int length);

int main()
{  int source;
   int obj;
   int length;
   long decimal_num;
   char temp[Maxchar];
   int flag = 1;
   while(flag)
   {  printf("The origional number is:");
      scanf("%s",temp);
      printf("The origional system is:");
      scanf("%d",&source);
      printf("The intended system to transfer is:");
      scanf("%d",&obj);
      printf("The transfered number is:");
      decimal_num = source2decimal(temp, source);
      length = decimal2obj(temp, decimal_num, obj);
      output(temp, length);
      printf("Continue-1;  Stop-0\n");
      scanf("%d",&flag);
   }
   return 0;
   
}

int char2num(char ch)        // pay attention the ASCII
{  if(ch>='0' && ch<='9')
     return ch-'0';
   else if(ch>='A' && ch<='F')
     return ch-'A';
   else if(ch>='a' && ch<='f')
     return ch-'a';
}

char num2char(int num)            // pay attention the format transfer to char
{  if(num>=0 && num<=9)
     return (char)(num+'0'); 
   else if(num>='A' && num<='F')
     return (char)(num+'A'-10);
   else if(num>='a' && num<='f')
     return (char)(num+'a'-10);
}

long source2decimal(char temp[], int source)
{  long decimal_num = 0;
   int length;
   int i;
   for(i=0; temp[i]!='\0'; i++)
     length = i;
   for(i=0; i<=length; i++)  
     decimal_num = (decimal_num*source) + char2num(temp[i]);
    return decimal_num;
}

int decimal2obj(char temp[], long decimal_num, int obj)
{  int i = 0;
   while(decimal_num)
   {  
      temp[i] = num2char(decimal_num % obj);
      decimal_num = decimal_num / obj;
      i++;
   }
   temp[i] ='\0';
   return i;
}

void output(char temp[], int length)
{  int i;
   for(i=length-1; i>=0; i--)
   printf("%c", temp[i]);
   printf("\n");
}
