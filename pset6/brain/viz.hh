#ifndef VIZ_H
#define VIZ_H

// representation of a single discretized cell
class Cell {
    public:

        Cell(): num_hits(0), prob(0) {};

        int num_hits; // counts number of times cell has been hit
        double prob; // likelihood that cell is occupied

        // void set_values(int, double);
        // double get_prob() { return prob; }
};

// void Cell::set_values (int num_hits, double prob) {
//   num_hits = num_hits;
//   prob = prob;
// }

int viz_run(int argc, char **argv);
void viz_hit(float range, float angle);
void viz_pos(Cell[][280]);

#endif
