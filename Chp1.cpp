#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct renode *Rectangle;
typedef struct renode
{
	int x, y, h, w;
} Renode;

int main()
{
	Renode rr;
	Rectangle R;
	R = &rr;
	rr.x = 1; rr.y = 1; rr.h = 5; rr.w = 5;
	printf("x= %d  y= %d\n", R->x, R->y);
	printf("Height= %d  Width= %d\n", R->h, R->w);

	char *str;
	if ((str = (char*)malloc(10)) == 0)
	{
		printf("Memory allocation faliure!\n ");
		exit(1);
	}
	strcpy(str, "Hello");
	printf("String is %s\n", str);
	free(str);
}