#ifndef __BUTTON__
#define __BUTTON__

typedef void (*button_Callback)();

void button_initialize();
void button_set_callback(button_Callback new_callback);

#endif
