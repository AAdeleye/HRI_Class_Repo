#ifndef TURN_TO_TRAINER_H
#define TURN_TO_TRAINER_H

#include <vector>
#include <cmvision/Blobs.h>

void trainer_blobs_callback (const cmvision::Blobs& blobsIn);

bool check_see_trainer();

int direction_to_turn();

#endif
