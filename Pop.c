#include <stdio.h>
int main()
{  
	int input[50] = {0}, 
       	    num = 0,
	    isOrder = 0,
	    i, j, temp;
	   
   	printf("Please input the array:\n");
   	do{  
	    scanf("%d",&input[num]);
            num++;
	 }
	 while ((getchar())!='\n');    // !!! 注意用 do-while 结构来接收输入，这样第一个数据就可以被存入数组，而while放在前的话
	                               // 第一个数据被作为getchar()输入不能被数组接收。 
	 
	printf("The  original array is:\n"); 
    	for(i=0; i<num; i++)
	printf("%d  ",input[i]);
	printf("\n");
	
	for(j=1; j<num; j++){ 
	    if (isOrder == 0){
	    for(i=0; i<num-j; i++){
	        isOrder = 1;          // 为提高冒泡程序效率，加入判断，无交换发生表明已经是顺序结构，无需再进行比较，每次都要初始化 
		if (input[i]>input[i+1])
	        {  temp = input[i+1];
	           input[i+1] = input[i];
	           input[i] = temp;
	           isOrder = 0;     // 有操作发生，还不是顺序结构 
	    } 
	    }
	  }
	  else
	    break;
	}
	
	printf("The array in ascending order is:\n");
        for(i=0; i<num; i++)
	printf("%d  ",input[i]);
	printf("\n");
		
}
