#ifndef SRC_OCCUPANCY_GRID_H
#define SRC_OCCUPANCY_GRID_H
#endif //SRC_OCCUPANCY_GRID_H

#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

#define THRESH 50

using namespace std;

namespace occupancy_grid{
    int xy_ind2ind(const nav_msgs::OccupancyGrid& grid, int x_ind, int y_ind){
        double width = int(grid.info.width);
        double n = int(grid.data.size());
        return min(y_ind * width + x_ind, n-1);
    }

    int xy2ind(const nav_msgs::OccupancyGrid& grid, float x, float y){
        // cood to grid idx
        double x0 = grid.info.origin.position.x;
        double y0 = grid.info.origin.position.y;

        double resolution = grid.info.resolution;

        int x_ind = (ceil((x-x0)/resolution))-1;
        int y_ind = (ceil((y-y0)/resolution))-1;
        return xy_ind2ind(grid, x_ind, y_ind);
    }

    struct Pair{
        int x_ind;
        int y_ind;
    };

    Pair ind2xy_ind(const nav_msgs::OccupancyGrid& grid, int ind){
        double width = grid.info.width;

        Pair res;
        res.y_ind = ind/width;
        res.x_ind = ind - res.y_ind*width;
        
        return res;
    }

    float ind2x(const nav_msgs::OccupancyGrid& grid, int ind){
        double x0 = grid.info.origin.position.x;
        Pair res = ind2xy_ind(grid, ind);
        double resolution = grid.info.resolution;
        return  x0 + res.x_ind * resolution;
    }

    float ind2y(const nav_msgs::OccupancyGrid& grid, int ind){
        double x0 = grid.info.origin.position.y;
        Pair res = ind2xy_ind(grid, ind);
        double resolution = grid.info.resolution;
        return  x0 + res.y_ind * resolution;
    }

    bool is_xy_occupied(nav_msgs::OccupancyGrid& grid, float x, float y){
        if (int(grid.data.at(xy2ind(grid, x, y))) > THRESH) {
            return true;
        }
        else {
            return false;
        }
    }

    bool is_xy_free(nav_msgs::OccupancyGrid& grid, float x, float y){
        return !is_xy_occupied(grid, x, y);
    }

    void set_xy_occupied(nav_msgs::OccupancyGrid& grid, float x, float y){
        grid.data.at(xy2ind(grid, x, y)) = 100;
    }

    void set_xy_free(nav_msgs::OccupancyGrid& grid, float x, float y){
        grid.data.at(xy2ind(grid, x, y)) = 0;
    }

    void inflate_cell(nav_msgs::OccupancyGrid &grid, int i, float margin, int val) {
        int margin_cells = ceil(margin/grid.info.resolution);
        Pair res = ind2xy_ind(grid, i);

        int x;
        int y;
        if (res.x_ind > margin_cells) {
            x = res.x_ind - margin_cells;
        }
        else x = 0;
        if (res.y_ind > margin_cells) {
            y = res.y_ind - margin_cells;
        }
        else y = 0;

        int width = int(grid.info.width);
        int height = int(grid.info.height);

        for (; x<min(width-1, res.x_ind+margin_cells); x++){
            for (; y<min(height-1, res.y_ind+margin_cells); y++){

                if (grid.data.at(xy_ind2ind(grid,x,y))>THRESH) continue;
                grid.data.at(xy_ind2ind(grid,x,y)) = val;
            }
        }
    }

    void inflate_obstacles(nav_msgs::OccupancyGrid& grid, float margin){
        vector<int> occupied_ind;
        occupied_ind.clear();

        for (int i=0; i<grid.data.size(); i++){
            if (grid.data.at(i)>THRESH){
                occupied_ind.push_back(i);
            }
        }
        for (int i=0; i<occupied_ind.size(); i++){
            inflate_cell(grid, occupied_ind[i], margin, 100);
        }
    }

}