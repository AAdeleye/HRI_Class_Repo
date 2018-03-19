#include "../include/turn_to_trainer/turn_to_trainer.h"

double goal_x;
bool see_trainer = false;

void trainer_blobs_callback (const cmvision::Blobs& blobsIn) {
    if (blobsIn.blob_count > 0) {
        see_trainer = true;
        double goal_sum_x = 0;
        for (int i = 0; i < blobsIn.blob_count; i++) {
            goal_sum_x += blobsIn.blobs[i].x;
        }
        goal_x = goal_sum_x / blobsIn.blob_count;
    } else {
        see_trainer = false;
    }
}

bool check_see_trainer() {
    return see_trainer;
}

int direction_to_turn() {
    double Center = 320;
    double error = Center - goal_x;
    if (error > 30.0) 
        return 1;
    else if (error < -30.0) 
        return -1;
    else
        return 0;
}

int main (int argc, char** argv)
{
    return 0;
}
