typedef void (*func_ptr)(void);

extern func_ptr __init_array_start[];
extern func_ptr __init_array_end[];
extern func_ptr __fini_array_start[];
extern func_ptr __fini_array_end[];

extern char __bss_start[];
extern char __bss_end[];

extern "C" void _init(void) {
  for (func_ptr *p = __init_array_start; p < __init_array_end; ++p) {
    if (*p)
      (*p)();
  }
}

extern "C" void _fini(void) {
  for (func_ptr *p = __fini_array_end; p != __fini_array_start;) {
    --p;
    if (*p)
      (*p)();
  }
}

extern "C" void bss_zero(void) {
  __builtin_memset(__bss_start, 0, __bss_end - __bss_start);
}
