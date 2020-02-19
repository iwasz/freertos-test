#ifndef ITOA_TEST_H
#define ITOA_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* itoa:  convert n to characters in s */
void itoa (int n, char s[], int zeroPad);
void itoau (unsigned int n, char s[], int zeroPad);

#ifdef __cplusplus
}
#endif

#endif
