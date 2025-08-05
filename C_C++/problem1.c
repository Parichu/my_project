#include <stdio.h>
#include <string.h>

#define ll long long
#define length 100


char a[length];


void runcase(){
  printf("Enter a string: ");
  fgets(a, sizeof(a), stdin);
  a[strcspn(a, "\n")] = 0; // remove newline
  printf("Length: %lu\n", strlen(a));

}


int main(){

  runcase();

  return 0;
}

