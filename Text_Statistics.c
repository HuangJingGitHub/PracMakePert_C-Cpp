// 基本延续C语言中文网提供的代码，逻辑判断部分稍有不同，统计一个文本
// 的信息，尤其注意注释的细节 
#include <stdio.h>
#include <string.h>

int *getCharNum(char *filename, int *totalNum);

int main()
{  char filename[30];
   int totalNum[3] = {0,0,0};
   
   printf("Input the file name:\n");
   scanf("%s",filename);
   
   if(getCharNum(filename, totalNum))
     printf("Total: %d line(s);  %d word(s);  %d char(s)", totalNum[0], totalNum[1], totalNum[2]);
   else
     printf("Error！\n");
    
    return 0;
}

int *getCharNum(char *filename, int *totalNum)
{  FILE *fp;
   char buffer[1003], c;
   int bufferlen, i,lineNum = 0, isLastBlank = 1, charNum = 0, wordNum = 0;
   
   if((fp=fopen(filename, "rb")) == NULL)
   {  perror(filename);
      return NULL;
   }
   
   printf("Line Words Chars\n");
   while(fgets(buffer, 1003, fp)!= NULL)         // 函数fgets()的三个参数 
   {  bufferlen = strlen(buffer);
      for(i=0; i<bufferlen; i++)
      {  c = buffer[i];
         if(c != ' ' && c !='\n' && c !='\r')   // 注意Windows系统中文本一行结束标志是\n\r, 换行加光标移动至行首 
         {  charNum++;                          // 非空格，非换行，则字符数加1 
            isLastBlank = 0;
         } 
         else if(c == ' ' && (!isLastBlank))
         {  wordNum++;
            isLastBlank = 1;
		 }
	  }
      
      !isLastBlank && wordNum++;             // 判断行尾是否以空格结束，不是以空格结束则单词数要加1
	                                         // 同时！a && b的运算规则相当于 if(!a) b, 
	  totalNum[0] ++;
	  totalNum[1] += wordNum;
	  totalNum[2] += charNum;
	  printf("%-7d%-7d%d\n", totalNum[0], wordNum, charNum);
	  
	  charNum = 0;                          // 重新初始化 
	  wordNum = 0;
	  isLastBlank = 1;
   }
   return totalNum;
}
