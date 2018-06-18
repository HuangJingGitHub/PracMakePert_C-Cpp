#include <stdio.h>
int main()
{ 
  int p1, p2, p5, counter=0;
  
  for (p5=0; p5<=100/5; p5++)            // Interesting
    {  for (p2=0; 2*p2<=100-p5*5; p2++)
         {  p1 = 100-5*p5-2*p2;
            printf("RMB 1yuan = %-2d 1fen + %-2d 2fen + %-2d 5fen\n",p1,p2,p5);
            counter++;
		 }
	}
  printf("\nThere are totally %d combinations!\n",counter);
}
