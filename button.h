#ifndef __BUTTON__
#define __BUTTON__

typedef void (*button_callback)();

void button_initialize();
void button_set_callback(button_callback new_callback);

#endif
