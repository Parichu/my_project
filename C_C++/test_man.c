#include <stdio.h>
#include <string.h>
#include <ctype.h>

int main() {
    char str1[100], str2[100], result[200];
    int choice;

    printf("=== String Operations in C ===\n");
    printf("1. String Length\n");
    printf("2. String Copy\n");
    printf("3. String Concatenation\n");
    printf("4. String Comparison\n");
    printf("5. String to Uppercase\n");
    printf("6. String to Lowercase\n");
    printf("7. Reverse String\n");
    printf("Enter your choice: ");
    scanf("%d", &choice);
    getchar(); // clear buffer

    switch(choice) {
        case 1: // String Length
            printf("Enter a string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0; // remove newline
            printf("Length: %lu\n", strlen(str1));
            break;

        case 2: // String Copy
            printf("Enter source string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;
            strcpy(str2, str1);
            printf("Copied string: %s\n", str2);
            break;

        case 3: // String Concatenation
            printf("Enter first string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;

            printf("Enter second string: ");
            fgets(str2, sizeof(str2), stdin);
            str2[strcspn(str2, "\n")] = 0;

            strcpy(result, str1);
            strcat(result, " ");
            strcat(result, str2);
            printf("Concatenated: %s\n", result);
            break;

        case 4: // String Comparison
            printf("Enter first string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;

            printf("Enter second string: ");
            fgets(str2, sizeof(str2), stdin);
            str2[strcspn(str2, "\n")] = 0;

            int cmp = strcmp(str1, str2);
            if (cmp == 0) {
                printf("Strings are equal\n");
            } else if (cmp < 0) {
                printf("First string is smaller\n");
            } else {
                printf("First string is larger\n");
            }
            break;

        case 5: // To Uppercase
            printf("Enter a string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;

            for (int i = 0; str1[i]; i++) {
                str1[i] = toupper(str1[i]);
            }
            printf("Uppercase: %s\n", str1);
            break;

        case 6: // To Lowercase
            printf("Enter a string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;

            for (int i = 0; str1[i]; i++) {
                str1[i] = tolower(str1[i]);
            }
            printf("Lowercase: %s\n", str1);
            break;

        case 7: // Reverse String
            printf("Enter a string: ");
            fgets(str1, sizeof(str1), stdin);
            str1[strcspn(str1, "\n")] = 0;

            int len = strlen(str1);
            for (int i = 0; i < len / 2; i++) {
                char temp = str1[i];
                str1[i] = str1[len - 1 - i];
                str1[len - 1 - i] = temp;
            }
            printf("Reversed: %s\n", str1);
            break;

        default:
            printf("Invalid choice!\n");
    }

    return 0;
}
