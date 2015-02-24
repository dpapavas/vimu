#ifndef _FUSION_H_
#define _FUSION_H_

typedef void (*fusion_data_ready_callback)(float *samples);

void fusion_set_callback(fusion_data_ready_callback new);
int fusion_in_progress();

#endif
