#include<stdio.h>

void swap(int *p1,int *p2)
{

  int t=*p1;
  *p1=*p2;
  *p2=t;
}

void permutation(int a[],int index,int size)
{
   int i, j;
   if(index==size)
   {
     for(i=0;i<size;i++)
     printf("%d ",a[i]);
     printf("\n"); 
   }
   else
   {
    for(j=index;j<size;j++)
    {
      swap(&a[j],&a[index]);
      permutation(a,index+1,size);  //此处用到递归思想
      swap(&a[j],&a[index]);
    }
  }
}

int main()
{   
   int i, n;
   scanf("%d",&n);
   int a[n];
   for(i=0;i<n;i++)
   a[i]=i+1;
   permutation(a,0,n);
   return 0;
}
