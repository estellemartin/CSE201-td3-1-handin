#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos

using namespace std;

using namespace support;
//
double* extend_array(double* array, int length, int new_size) { //double* array: array is a pointer to an array of doubles
  double* newarray= new double[new_size];
  //double newa[new_size];
  //double* newarray = newa;
  for(int i=0; i< new_size; i++){
      if(i<length){
        newarray[i] = array[i];
      }
      else{
          newarray[i]=0;
      }
  }
      delete[] array;
      //  double p;
      //newarray* p;
      //double p = &newarray;e
      return newarray;
}

double* shrink_array(double* array, int length, int new_size) {
  double* newarray= new double[new_size];
  for(int i=0; i<new_size;i++){
      newarray[i]=array[i];
  }
  delete[] array;
  return newarray;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size, //& means that current_size is passed by reference so when you modify it it is modified in memory (deep) otherwise it is modified in heap so it kind of cancels at the end of the function
                        int &max_size) {

    if(current_size!=max_size){
        //array[-1]=element;
       *(array+current_size) = element;
    }
    else{
      max_size+=5;
      array = extend_array(array, current_size, max_size);
      //array[current_size+1]=element;
      *(array+current_size) = element;
    }

    current_size += 1;
  return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    // shrinks when the difference between the **total number of used elements total_elements**
    // and the array maximum size **array_size** is at least 5
    if (current_size > 0){
        current_size--;
        if (max_size - current_size >= 5){
            array = shrink_array(array, max_size, max_size-5);
            max_size -= 5;
        }
    }
  return array;
}
bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
      telemetry = append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &telemetry,
                     int &telemetry_current_size,
                     int &telemetry_max_size) {
 for (int i = 0; i<tot_telemetries; i++ ){
     for (int j = 0; j<telemetries_sizes[i];j++ ){
         telemetry = append_to_array(telemetries[i][j],telemetry,telemetry_current_size,telemetry_max_size);
     }
 }
 for (int i = 0; i < telemetry_current_size/3-1; i++)
 {
     for (int j = 0; j < telemetry_current_size/3-i-1; j++)
     {
         if ((telemetry[3*j]>telemetry[3*j+3]))
         {
           double t = telemetry[3*j+3];
           telemetry[3*j+3] = telemetry[3*j];
           telemetry[3*j] = t;
           double x = telemetry[3*j+4];
           telemetry[3*j+4] = telemetry[3*j+1];
           telemetry[3*j+1] = x;
           double y = telemetry[3*j+5];
           telemetry[3*j+5] = telemetry[3*j+2];
           telemetry[3*j+2] = y;
         }
     }
 }
}
