#include <stdio.h>
#include <math.h>

void circle(double r, double angle_step_in_deg, double how_much_to_raise, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT %.12lf 0.0 %.12lf 0.0\n", r, original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / portion_of_circle * total_time + 0.5);
      double new_x = r * cos(angle);
      double new_y = r * sin(angle);
      double new_height = original_height + angle / portion_of_circle * how_much_to_raise;
      double new_angle = angle;
      while (new_angle > 2 * M_PI) new_angle -= 2 * M_PI;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      new_angle); 

    }

      int time_in_ms = (int)(total_time + 0.5);
      double new_x = r * cos(portion_of_circle);
      double new_y = r * sin(portion_of_circle);
      double new_height = original_height + how_much_to_raise;
      double new_angle = portion_of_circle;
      while (new_angle > 2 * M_PI) new_angle -= 2 * M_PI;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      new_angle); 

    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleCW(double r, double angle_step_in_deg, double how_much_to_raise, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT %.12lf 0.0 %.12lf 3.1415926536\n", r, original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_x = r * cos(angle);
      double new_y = -r * sin(angle);
      double new_height = original_height + angle / (portion_of_circle) * how_much_to_raise;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      M_PI - angle); 

    }

      int time_in_ms = (int)(total_time + 0.5);
      double new_x = r * cos(portion_of_circle);
      double new_y = - r * sin(portion_of_circle);
      double new_height = original_height + how_much_to_raise;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      M_PI - portion_of_circle); 

    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleV(double r, double angle_step_in_deg, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT 0.8 0.0 %.12lf 0.0\n", original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_height = r * cos(angle);
      double new_y = -r * sin(angle);

      fprintf(f, "%d  POS  0.8 %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_y,
		      original_height - r + new_height,
		      3.14); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleVR(double r, double angle_step_in_deg, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT -0.8 0.0 %.12lf 0.0\n", original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_height = r * cos(angle);
      double new_y = r * sin(angle);

      fprintf(f, "%d  POS  -0.8 %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_y,
		      original_height - r + new_height,
		      0.0); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void vertical_movement(double starting_height, double final_height, double starting_heading, double heading_total_increment, double total_time, int number_of_steps, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);

    for (int i = 0; i < number_of_steps; i++)
    {
      int time_in_ms = (int)(total_time / number_of_steps * i + 0.5); 
      double current_height = starting_height + (1 + i) * (final_height - starting_height) / number_of_steps;
      double current_heading = starting_heading + (1 + i) * heading_total_increment / number_of_steps;

      fprintf(f, "%d  POS  0.01 0.01 %.12lf %.12lf\n", 
		      time_in_ms,
		      current_height,
		      current_heading);
    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

int main()
{
    double r = 0.95;
    double angle_step_in_deg = 5.0;
    double how_much_to_raise = 1.7 / 33 * 11;
    double original_height = 1.7; 
    double total_time = 16000;
    double portion_of_circle = 2 * M_PI;

    circleCW(r, angle_step_in_deg, how_much_to_raise, original_height, total_time * 11.0 / 16.0, portion_of_circle * 11.0 / 16.0, "RIDE_11_16");
    circle(r, angle_step_in_deg, -how_much_to_raise, 3.4, total_time * 11.0 / 16.0, portion_of_circle * 11.0 / 16.0, "REVERSE_RIDE_11_16");


    circleV(0.5, angle_step_in_deg, 3.5, 10000, portion_of_circle, "VERTICAL_LOOP_SOUTH");
    circleVR(0.5, angle_step_in_deg, 3.5, 10000, portion_of_circle, "VERTICAL_LOOP_NORTH");
    circleVR(0.5, angle_step_in_deg, 3.5, 6000, portion_of_circle * 6 / 10, "VERTICAL_HALF_LOOP_NORTH");
    
    circle(0.5, angle_step_in_deg, 0.0, 2.15, 10000, portion_of_circle, "HORIZONTAL_LOOP_CCW");
    circle(0.5, angle_step_in_deg, 0.0, 2.15, 6000, portion_of_circle * 6 / 10, "HORIZONTAL_HALF_LOOP_CCW");
    circleCW(0.5, angle_step_in_deg, 0.0, 1.65, 10000, portion_of_circle, "HORIZONTAL_LOOP_CW");

    vertical_movement(3.4, 1.7, 3.14, 0.0, 6000, 25, "FLY_DOWN");
    vertical_movement(1.7, 3.4, 0.0, 0.0, 6000, 25, "FLY_UP");
}



