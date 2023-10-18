//
// Author: Yuwei Wang


#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

#define THRESHOLD 50

#define inaccessible_cost_ 100
#define MAP_UNREACHABLE 100

using namespace std;

namespace occupancy_grid{
    int xy_ind2ind(const nav_msgs::OccupancyGrid& grid, int x_ind, int y_ind){
        return max(0, min(y_ind * int(grid.info.width) + x_ind, int(grid.data.size())-1));
    }

    int xy2ind(const nav_msgs::OccupancyGrid& grid, float x, float y){
        int x_ind = static_cast<int>(ceil((x-grid.info.origin.position.x)/grid.info.resolution))-1;
        int y_ind = static_cast<int>(ceil((y-grid.info.origin.position.y)/grid.info.resolution))-1;
        return xy_ind2ind(grid, x_ind, y_ind);
    }

    struct Pair{
        int x_ind;
        int y_ind;
    };

    Pair ind2xy_ind(const nav_msgs::OccupancyGrid& grid, int ind){
        int y_ind = ind/grid.info.width;
        int x_ind = ind - y_ind*grid.info.width;
        Pair res;
        res.x_ind = x_ind;
        res.y_ind = y_ind;
        return res;
    }

    float ind2x(const nav_msgs::OccupancyGrid& grid, int ind){
        return grid.info.origin.position.x + ind2xy_ind(grid, ind).x_ind * grid.info.resolution;
    }

    float ind2y(const nav_msgs::OccupancyGrid& grid, int ind){
        return grid.info.origin.position.y + ind2xy_ind(grid, ind).y_ind * grid.info.resolution;
    }

    bool is_xy_occupied(nav_msgs::OccupancyGrid& grid, float x, float y){
        return int(grid.data.at(xy2ind(grid, x, y))) > THRESHOLD;
    }

    void set_xy_occupied(nav_msgs::OccupancyGrid& grid, float x, float y){
        grid.data.at(xy2ind(grid, x, y)) = 100;
    }

    void inflate_cell(nav_msgs::OccupancyGrid &grid, int i, float margin, int val) {
        int margin_cells = static_cast<int>(ceil(margin/grid.info.resolution));
        Pair res = ind2xy_ind(grid, i);
        for (int x = max(0, res.x_ind-margin_cells); x<min(int(grid.info.width-1), res.x_ind+margin_cells); x++){
            for (int y = max(0, res.y_ind-margin_cells); y<min(int(grid.info.height-1), res.y_ind+margin_cells); y++){
                grid.data.at(xy_ind2ind(grid,x,y)) = val;
            }
        }
    }
    int GetIndex(const nav_msgs::OccupancyGrid &map_, int mx,   int my) 
    {
        return my*map_.info.width+mx;
    }

    int GetCost(const nav_msgs::OccupancyGrid &map_, int mx,  int my)
    {
        if (mx<0||mx>map_.info.width)
            return  inaccessible_cost_;
        if(my<0||my>map_.info.height)
            return  inaccessible_cost_;
        return map_.data[GetIndex(map_,mx, my)];
    }

    void inflate_map(nav_msgs::OccupancyGrid& grid, float margin)
    {
        vector<int> occupied_ind;
        occupied_ind.clear();
        for (int i=0; i<grid.data.size(); i++){
            if (grid.data.at(i)>THRESHOLD){
                int tmp_x = 0;
                int tmp_y = 0;
                tmp_x=ind2xy_ind(grid,i).x_ind;
                tmp_y=ind2xy_ind(grid,i).y_ind;
                bool isSurrounded = true;
                for (int dx=-1; dx<=1; dx++) 
                {
                    int nx = tmp_x+dx;
                    if (nx<0 || nx>=grid.info.width) 
                    {
                        continue;
                    }
                    for (int dy=-1; dy<=1; dy++)
                    {
                        // if (dx==0 && dy==0) 
                        // {
                        //     continue;
                        // }
                        int ny = tmp_y+dy;
                        if (ny<0 || ny>=grid.info.height) 
                        {
                            continue;
                        }
                        if (grid.data.at(xy_ind2ind(grid,nx,ny))<THRESHOLD) 
                        {
                            isSurrounded = false;
                            break;
                        }
                    }
                    if(!isSurrounded)
                    {
                        break;
                    }
                }
                if(!isSurrounded)
                {
                    occupied_ind.push_back(i);
                }
            }
        }
        // cout<<"----index size-----="<<occupied_ind.size()<<endl;
        for (int i=0; i<occupied_ind.size(); i++){
            inflate_cell(grid, occupied_ind[i], margin, 100);
        }


        // int radius2cells=static_cast<int>(ceil(margin/grid.info.resolution));

        // radius2cells=1;
   
        // int tmp_x = 0;
        // int tmp_y = 0;
        // while(tmp_y < grid.info.height)
        // {
        //     tmp_x=0;
        //     while(tmp_x < grid.info.width)
        //     {
        //         if(GetCost(grid,tmp_x,tmp_y)==MAP_UNREACHABLE)
        //         {
                    
        //             bool isSurrounded = true;
        //             for (int dx=-1; dx<=1; dx++) 
        //             {
        //                 int nx = tmp_x+dx;
        //                 if (nx<0 || nx>=grid.info.width) 
        //                 {
        //                     continue;
        //                 }
        //                 for (int dy=-1; dy<=1; dy++)
        //                 {
        //                     if (dx==0 && dy==0) 
        //                     {
        //                         continue;
        //                     }
        //                     int ny = tmp_y+dy;
        //                     if (ny<0 || ny>=grid.info.height) 
        //                     {
        //                         continue;
        //                     }
        //                     if (GetCost(grid,nx,ny)!=MAP_UNREACHABLE) 
        //                     {
        //                         isSurrounded = false;
        //                         break;
        //                     }
        //                 }
        //                 if(!isSurrounded)
        //                 {
        //                     break;
        //                 }
        //             }
        //             if(!isSurrounded)
        //             {
        //                 for(int i=tmp_x-radius2cells;i<=tmp_x+radius2cells;i++) 
        //                 {
        //                     for (int j = tmp_y - radius2cells; j <= tmp_y + radius2cells; j++)
        //                     {
        //                         if ((j >= 0 && j < grid.info.height) && (i >= 0 && i < grid.info.width))
        //                         {
        //                             cout<<"----tmp----"<<tmp_x<<" "<<tmp_y<<" "<<i<<" "<<j<<endl;
        //                             cout<<"---x----="<<ind2x(grid,xy_ind2ind(grid,i,j))<<" "<<ind2y(grid,xy_ind2ind(grid,i,j))<<endl;
        //                             grid.data.at(xy_ind2ind(grid,i,j)) = 100;
                                    
        //                         }
        //                     }
        //                 }
        //             } 
                    
        //         }
        //         tmp_x += 1;
        //     }
        //     tmp_y += 1;
        // }

        // cout<<"----fusion map is over----"<<endl;
    }
}