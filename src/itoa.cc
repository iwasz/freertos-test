#include

/* reverse:  reverse string s in place */
static void reverse (char s[])
{
        int i, j;
        char c;

        for (i = 0, j = strlen (s) - 1; i < j; i++, j--) {
                c = s[i];
                s[i] = s[j];
                s[j] = c;
        }
}

/* itoa:  convert n to characters in s */
void itoa (int n, char s[], int zeroPad)
{
        int i, sign;

        if ((sign = n) < 0) { /* record sign */
                n = -n;       /* make n positive */
        }

        i = 0;

        do {                           /* generate digits in reverse order */
                s[i++] = n % 10 + '0'; /* get next digit */
        } while ((n /= 10) > 0);       /* delete it */

        for (; i < zeroPad; ++i) {
                s[i] = '0';
        }

        if (sign < 0) {
                s[i++] = '-';
        }

        s[i] = '\0';

        reverse (s);
}

void itoa (unsigned int n, char s[], int zeroPad)
{
        int i;

        i = 0;

        do {                           /* generate digits in reverse order */
                s[i++] = n % 10 + '0'; /* get next digit */
        } while ((n /= 10) > 0);       /* delete it */

        for (; i < zeroPad; ++i) {
                s[i] = '0';
        }

        s[i] = '\0';

        reverse (s);
}
