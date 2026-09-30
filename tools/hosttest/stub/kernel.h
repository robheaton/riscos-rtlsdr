#include <stdint.h>
typedef struct { int r[10]; } _kernel_swi_regs;
typedef struct { int errnum; char errmess[252]; } _kernel_oserror;
static inline _kernel_oserror *_kernel_swi(int n, _kernel_swi_regs *a, _kernel_swi_regs *b) { (void)n;(void)a;(void)b; return 0; }
